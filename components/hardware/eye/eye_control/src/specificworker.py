#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from PySide2.QtGui import QImage, QPixmap
from PySide2.QtCharts import QtCharts
from PySide2.QtCore import QRandomGenerator
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import numpy as np

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

QChartView = QtCharts.QChartView
QChart = QtCharts.QChart
QValueAxis = QtCharts.QValueAxis
QLineSeries = QtCharts.QLineSeries

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        motor = self.jointmotorsimple_proxy.getMotorState("")
        self.ui.horizontalSlider_pos.setSliderPosition(motor.pos)
        self.current_max_speed = 0.0

        QObject.connect(self.ui.horizontalSlider_pos, SIGNAL('valueChanged(int)'), self.slot_change_pos)
        QObject.connect(self.ui.horizontalSlider_max_speed, SIGNAL('valueChanged(int)'), self.slot_change_max_speed)
        QObject.connect(self.ui.pushButton_center, SIGNAL('clicked()'), self.slot_center)

        self.m_x = 0
        self.chart = QChart()
        self.track_error = QLineSeries()
        green_pen = QPen(Qt.blue)
        green_pen.setWidth(2)
        self.track_error.setPen(green_pen)
        self.axisX = QValueAxis()
        self.axisY = QValueAxis()
        self.axisX.setTickCount(0.1)
        self.axisX.setRange(0, 1000)
        self.axisY.setRange(-5, 10)
        self.chart.addSeries(self.track_error)
        self.chart.addAxis(self.axisX, Qt.AlignBottom)
        self.chart.addAxis(self.axisY, Qt.AlignLeft)
        self.track_error.attachAxis(self.axisX)
        self.track_error.attachAxis(self.axisY)
        self.chart.legend().hide()
        self.chartView = QChartView(self.chart, self.ui.frame_error)
        self.chartView.resize(self.ui.frame_error.size())
        self.chartView.show()

        self.Period = 100
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True


    @QtCore.Slot()
    def compute(self):
        #print('SpecificWorker.compute...')
        try:
            motor = self.jointmotorsimple_proxy.getMotorState("")
        except:
            print("Ice error communicating with JointMotorSimple interface")

        try:
            color = self.camerasimple_proxy.getImage()
            #image = np.frombuffer(color.image, np.uint8).reshape(color.height, color.width, color.depth)
        except:
            print("Ice error communicating with CameraSimple interface")

        image = QImage(color.image, color.width, color.height, QImage.Format_RGB888)
        pix = QPixmap.fromImage(image).scaled(self.ui.label_image.width(), self.ui.label_image.height())
        self.ui.label_image.setPixmap(pix)

        self.ui.lcdNumber_pos.display(motor.pos)
        self.ui.lcdNumber_speed.display(motor.vel)
        self.ui.lcdNumber_temp.display(motor.temperature)
        self.ui.lcdNumber_max_speed.display(self.current_max_speed)
        if motor.isMoving:
            self.ui.radioButton_moving.setChecked(True)
        else:
            self.ui.radioButton_moving.setChecked(False)

        self.graph_tick()

    @QtCore.Slot()
    def slot_change_pos(self, pos):   # comes in degrees -150 .. 150. Sent in radians -2.62 .. 2.62
        goal = ifaces.RoboCompJointMotorSimple.MotorGoalPosition()
        goal.position = (2.62/150.0)*pos
        goal.maxSpeed = self.current_max_speed
        self.jointmotorsimple_proxy.setPosition("", goal)

    @QtCore.Slot()
    def slot_change_max_speed(self, max_speed):
        self.current_max_speed = max_speed*0.111/60*np.pi*2.0

    @QtCore.Slot()
    def slot_center(self):
        self.ui.horizontalSlider_pos.setSliderPosition(0)

    def graph_tick(self):
        m_y = QRandomGenerator.global_().bounded(5) - 2.5
        self.m_x += 20
        self.track_error.append(self.m_x, m_y)
        if self.m_x > 1000:
            self.chart.scroll(5, 0)


#####################################################################################################
    def startup_check(self):
        print(f"Testing RoboCompJointMotorSimple.MotorState from ifaces.RoboCompJointMotorSimple")
        test = ifaces.RoboCompJointMotorSimple.MotorState()
        print(f"Testing RoboCompJointMotorSimple.MotorParams from ifaces.RoboCompJointMotorSimple")
        test = ifaces.RoboCompJointMotorSimple.MotorParams()
        print(f"Testing RoboCompJointMotorSimple.MotorGoalPosition from ifaces.RoboCompJointMotorSimple")
        test = ifaces.RoboCompJointMotorSimple.MotorGoalPosition()
        print(f"Testing RoboCompJointMotorSimple.MotorGoalVelocity from ifaces.RoboCompJointMotorSimple")
        test = ifaces.RoboCompJointMotorSimple.MotorGoalVelocity()
        QTimer.singleShot(200, QApplication.instance().quit)

    ######################
    # From the RoboCompJointMotorSimple you can call this methods:
    # self.jointmotorsimple_proxy.getMotorParams(...)
    # self.jointmotorsimple_proxy.getMotorState(...)
    # self.jointmotorsimple_proxy.setPosition(...)
    # self.jointmotorsimple_proxy.setVelocity(...)
    # self.jointmotorsimple_proxy.setZeroPos(...)

    ######################
    # From the RoboCompJointMotorSimple you can use this types:
    # RoboCompJointMotorSimple.MotorState
    # RoboCompJointMotorSimple.MotorParams
    # RoboCompJointMotorSimple.MotorGoalPosition
    # RoboCompJointMotorSimple.MotorGoalVelocity

