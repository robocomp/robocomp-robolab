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
from meld.vc import _null
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import numpy as np
import cv2, traceback, json, math
import time

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

QChartView = QtCharts.QChartView
QChart = QtCharts.QChart
QValueAxis = QtCharts.QValueAxis
QLineSeries = QtCharts.QLineSeries

MIN_VELOCITY = 0.05
TOLERANCE = 0.01
SERVO_TOLERANCE = 0.1

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        motor = self.jointmotorsimple_proxy.getMotorState("")
        self.ui.horizontalSlider_pos.setSliderPosition(motor.pos)
        self.current_max_speed = 0.0

        QObject.connect(self.ui.horizontalSlider_pos, SIGNAL('valueChanged(int)'), self.slot_change_pos)
        QObject.connect(self.ui.horizontalSlider_max_speed, SIGNAL('valueChanged(int)'), self.slot_change_max_speed)
        QObject.connect(self.ui.horizontalSlider, SIGNAL('valueChanged(int)'), self.slot_giraff_speed)
        QObject.connect(self.ui.pushButton_center, SIGNAL('clicked()'), self.slot_center)
        QObject.connect(self.ui.pushButton, SIGNAL('clicked()'), self.slot_track)
        QObject.connect(self.ui.pushButton_2, SIGNAL('clicked()'), self.slot_STOP)

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

        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

        with open('human_pose.json', 'r') as f:
            self.human_pose = json.load(f)

        self.track = False
        self.error_ant = 0
        self.rad_old = 0

        # Speed filters variables
        self.rotational_speed_coefficients=[0,0,0]
        self.rotational_speed_avg=None
        self.lineal_speed_coefficients=[0,0,0]
        self.lineal_speed_avg=None


        # Person distance filter
        self.distance_coefficients=[0,0,0,0]
        self.distance_avg=None

        # Distance limit
        self.distance_limit = 1300

        self.Period = 10
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
        #obtenemos datos
        motor, color, people_data = self.obtencion_datos()

        image = np.frombuffer(color.image, np.uint8).reshape(color.width, color.height, color.depth)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Draw body parts on image
        image, puntoMedioX, puntoMedioY, distance = self.proceso_esqueleto(image, people_data)

        #Hay persona y modo track activado
        if puntoMedioY and puntoMedioX and self.track and distance:
            self.tracker(color, motor, puntoMedioX, distance)

        #Actualización de ventana Eye Control
        self.refesco_ventana(color, image, motor)

    def obtencion_datos(self):
        try:
            motor = self.jointmotorsimple_proxy.getMotorState("")
        except:
            print("Ice error communicating with JointMotorSimple interface")

        try:
            color = self.camerargbdsimple_proxy.getImage("")
        except:
            print("Ice error communicating with CameraRGBDSimple interface")

        try:
            people_data = self.humancamerabody_proxy.newPeopleData()
            # print(list(people_data.peoplelist[0].joints.keys()))
        except:
            traceback.print_exc()
            print("Ice error communicating with HumaCameraBody interface")
        return motor, color, people_data

    def proceso_esqueleto(self, image, people_data):
        # Draw body parts on image
        for person in people_data.peoplelist:
            for name1, name2 in self.human_pose["skeleton"]:
                try:
                    # Printing bones
                    # if str(name1) in list(person.joints.keys()) and str(name2) in list(person.joints.keys()):
                    #    print(name1, name2)
                    joint1 = person.joints[str(name1)]
                    joint2 = person.joints[str(name2)]
                    # print(joint1.i, joint2.j)
                    cv2.line(image, (joint1.i, joint1.j), (joint2.i, joint2.j), (0, 255, 0), 2)
                except:
                    pass

        # compute bounding box
        faceList = ["2", "3"]
        hipList = ["12", "13"]
        faceNameList = []
        hipNameList = []

        if len(people_data.peoplelist) > 0:
            person = people_data.peoplelist[0]
            keypoints_x = []
            keypoints_y = []
            keypoints_z = []  # profundidad
            for key in person.joints:
                keypoint = person.joints[key]
                keypoints_x.append(keypoint.x)  # sacar las x de la posicion respecto al mundo
                keypoints_y.append(keypoint.y)
                keypoints_z.append(keypoint.z)  # sacar las y de la posicion respecto al mundo
            pos_x = sum(keypoints_x) / len(keypoints_x)
            pos_y = sum(keypoints_y) / len(keypoints_y)
            pos_z = sum(keypoints_z) / len(keypoints_z)
            distance_vector = [pos_x*1000, pos_y*1000, pos_z*1000]
            distance = distance_vector[2]
            print("Distance init: ", distance[2])

            for key_point in list(person.joints.keys()):
                if key_point in faceList:
                    faceNameList.append(key_point)
                if key_point in hipList:
                    hipNameList.append(key_point)
        else:
            distance = 0

        puntoMedioX = None
        puntoMedioY = None

        if len(faceNameList) == 2:
            i0 = person.joints[faceNameList[0]].i
            j0 = person.joints[faceNameList[0]].j
            i1 = person.joints[faceNameList[1]].i
            j1 = person.joints[faceNameList[1]].j
            puntoMedioX = (i0 + i1) / 2.0
            puntoMedioY = (j0 + j1) / 2.0
            cv2.circle(image, (int(puntoMedioX - 10), int(puntoMedioY - 10)), 10, (255, 0, 0), 2)
            cv2.rectangle(image, (int(i0), int(j0)), (int(i1), int(j1)), (255, 128, 0), 1)

            # points = np.array()
            # points = np.append(ooints, )
            # rect = cv2.boundingRect(points)
            # cv2.rectangle(image,  (int(i0), int(j0)), (int(i1), int(j1)), (255, 128, 0), 1)

            # error = puntoMedioX - color.width/2
            # goal = ifaces.RoboCompJointMotorSimple.MotorGoalPosition()
            # error_rads = np.arctan2(error, color.focalx)
            # print(goal.position, color.focalx, motor.pos)
            # goal.position = motor.pos - error_rads
            #     print(goal)
            # self.jointmotorsimple_proxy.setPosition("", goal)

        # Filtering rotational speeds
        self.distance_coefficients.append(distance)
        self.distance_coefficients.pop(0)
        self.distance_avg = sum(self.distance_coefficients) / len(self.distance_coefficients)

        if len(hipNameList) == 2:
            puntoMedioX = (person.joints[hipNameList[0]].i + person.joints[hipNameList[1]].i) / 2.0
            puntoMedioY = (person.joints[hipNameList[0]].j + person.joints[hipNameList[1]].j) / 2.0
            cv2.circle(image, (int(puntoMedioX - 10), int(puntoMedioY - 10)), 10, (0, 150, 255), 2)
        return image, puntoMedioX, puntoMedioY, self.distance_avg

    def tracker(self, color, motor, puntoMedioX, distance):
        print("Distance: ", distance)
        error = puntoMedioX - color.width / 2
        der_error = -(error - self.error_ant)
        error_rads = np.arctan2(1.1 * error + 0.8 * der_error, color.focalx)

        goal = ifaces.RoboCompJointMotorSimple.MotorGoalPosition()
        goal_rad = motor.pos - error_rads

        # Calculate and filter person speed
        if self.rad_old > goal_rad + TOLERANCE or self.rad_old < goal_rad - TOLERANCE:
            # Calculating rotational speed with image and period data
            rot_rad_seg = abs(((self.rad_old - goal_rad) / self.Period) * 1000)  # rad/s
        else:
            rot_rad_seg = 0

        # Set rotational base speed

        # Check if somebody is centered by the camera
        if abs(error_rads) < 0.05:
            print("CENTERED")
            if motor.pos > 0:
                base_speed_rot = -1*(1 - math.exp(-(error_rads ** 2)/5))
            elif motor.pos < 0:
                base_speed_rot = 1*(1 - math.exp(-(error_rads ** 2)/5))
            else:
                base_speed_rot = 0

        else:
            print("NOT CENTERED")
            if (self.rad_old > goal_rad):
                base_speed_rot = 2*(1 - math.exp(-(rot_rad_seg ** 2)/10))
            else:
                base_speed_rot = -2*(1 - math.exp(-(rot_rad_seg ** 2)/10))

        # Check if the camera is aligned with the robot
        if abs(motor.pos) <= 0.1:
            print("ALIGNED")
            base_speed_rot = 0

        # Filtering rotational speeds
        self.rotational_speed_coefficients.append(base_speed_rot)
        self.rotational_speed_coefficients.pop(0)
        self.rotational_speed_avg = sum(self.rotational_speed_coefficients)/len(self.rotational_speed_coefficients)

        # Set camera speed
        if abs(self.rotational_speed_avg) < rot_rad_seg:
            camera_speed = rot_rad_seg - self.rotational_speed_avg
            # filtramos velicidad, ya que 0 es maxima
            if camera_speed < MIN_VELOCITY:
                goal.maxSpeed = MIN_VELOCITY
            else:
                goal.maxSpeed = camera_speed
            goal.position = motor.pos - error_rads
            # mandamos al motor el objetivo
            self.jointmotorsimple_proxy.setPosition("", goal)

            # print(goal_rad, self.rad_old, rad_seg)

        self.error_ant = error
        self.rad_old = goal_rad

        # Calculating lineal speed with distance
        if distance < self.distance_limit or distance == 0:
            lin_rad_seg = 0
        else:
            lin_rad_seg = 1 - (self.distance_limit/distance)

        # Set lineal base speed. Rotational speed dependence
        base_speed_lin = 1000 * lin_rad_seg * math.exp(-(self.rotational_speed_avg ** 2)/10)

        # Filtering lineal speeds
        self.lineal_speed_coefficients.append(base_speed_lin)
        self.lineal_speed_coefficients.pop(0)
        self.lineal_speed_avg = sum(self.lineal_speed_coefficients) / len(self.lineal_speed_coefficients)

        print("base_speed_lin: ", self.lineal_speed_avg)
        print("base_speed_rot: ", self.rotational_speed_avg)

        # Sending speeds to base
        # self.differentialrobot_proxy.setSpeedBase(self.lineal_speed_avg, self.rotational_speed_avg)

    def refesco_ventana(self, color, image, motor):
        qt_image = QImage(image, color.height, color.width, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qt_image).scaled(self.ui.label_image.width(), self.ui.label_image.height())
        self.ui.label_image.setPixmap(pix)
        # image = np.frombuffer(color.image, np.uint8).reshape(color.height, color.width, color.depth)

        self.ui.lcdNumber_pos.display(motor.pos)
        self.ui.lcdNumber_speed.display(motor.vel)
        self.ui.lcdNumber_temp.display(motor.temperature)
        self.ui.lcdNumber_max_speed.display(self.current_max_speed)
        if motor.isMoving:
            self.ui.radioButton_moving.setChecked(True)
        else:
            self.ui.radioButton_moving.setChecked(False)

        # self.graph_tick()

    @QtCore.Slot()
    def slot_change_pos(self, pos):   # comes in degrees -150 .. 150. Sent in radians -2.62 .. 2.62
        goal = ifaces.RoboCompJointMotorSimple.MotorGoalPosition()
        goal.position = -(2.62/150.0)*pos
        goal.maxSpeed = self.current_max_speed
        self.jointmotorsimple_proxy.setPosition("", goal)

    @QtCore.Slot()
    def slot_change_max_speed(self, max_speed):
        self.current_max_speed = max_speed*0.111/60*np.pi*2.0

    @QtCore.Slot()
    def slot_center(self):
        self.ui.horizontalSlider_pos.setSliderPosition(0)
        self.slot_change_pos(0)

    @QtCore.Slot()
    def slot_track(self):
        self.track = not self.track
        self.differentialrobot_proxy.setSpeedBase(0, 0)
        print("state track", self.track)

    @QtCore.Slot()
    def slot_STOP(self):
        self.track = False
        self.differentialrobot_proxy.setSpeedBase(0, 0)
        print("¡¡¡¡¡¡STOOOOOOOOOOOOOP!!!!!!")

    @QtCore.Slot()
    def slot_giraff_speed(self, speed):
        self.differentialrobot_proxy.setSpeedBase(0, speed/1000)
        self.ui.horizontalSlider.setSliderPosition(0)


    def graph_tick(self):
        m_y = QRandomGenerator.global_().bounded(5) - 2.5
        self.m_x += 20
        self.track_error.append(self.m_x, m_y)
        if self.m_x > 1000:
            self.chart.scroll(5, 0)


#####################################################################################################
    def startup_check(self):
        print(f"Testing RoboCompCameraRGBDSimple.TImage from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TImage()
        print(f"Testing RoboCompCameraRGBDSimple.TDepth from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TDepth()
        print(f"Testing RoboCompCameraRGBDSimple.TRGBD from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TRGBD()
        print(f"Testing RoboCompDifferentialRobot.TMechParams from ifaces.RoboCompDifferentialRobot")
        test = ifaces.RoboCompDifferentialRobot.TMechParams()
        print(f"Testing RoboCompHumanCameraBody.TImage from ifaces.RoboCompHumanCameraBody")
        test = ifaces.RoboCompHumanCameraBody.TImage()
        print(f"Testing RoboCompHumanCameraBody.TGroundTruth from ifaces.RoboCompHumanCameraBody")
        test = ifaces.RoboCompHumanCameraBody.TGroundTruth()
        print(f"Testing RoboCompHumanCameraBody.KeyPoint from ifaces.RoboCompHumanCameraBody")
        test = ifaces.RoboCompHumanCameraBody.KeyPoint()
        print(f"Testing RoboCompHumanCameraBody.Person from ifaces.RoboCompHumanCameraBody")
        test = ifaces.RoboCompHumanCameraBody.Person()
        print(f"Testing RoboCompHumanCameraBody.PeopleData from ifaces.RoboCompHumanCameraBody")
        test = ifaces.RoboCompHumanCameraBody.PeopleData()
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
    # From the RoboCompCameraRGBDSimple you can call this methods:
    # self.camerargbdsimple_proxy.getAll(...)
    # self.camerargbdsimple_proxy.getDepth(...)
    # self.camerargbdsimple_proxy.getImage(...)

    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # RoboCompCameraRGBDSimple.TImage
    # RoboCompCameraRGBDSimple.TDepth
    # RoboCompCameraRGBDSimple.TRGBD

    ######################
    # From the RoboCompDifferentialRobot you can call this methods:
    # self.differentialrobot_proxy.correctOdometer(...)
    # self.differentialrobot_proxy.getBasePose(...)
    # self.differentialrobot_proxy.getBaseState(...)
    # self.differentialrobot_proxy.resetOdometer(...)
    # self.differentialrobot_proxy.setOdometer(...)
    # self.differentialrobot_proxy.setOdometerPose(...)
    # self.differentialrobot_proxy.setSpeedBase(...)
    # self.differentialrobot_proxy.stopBase(...)

    ######################
    # From the RoboCompDifferentialRobot you can use this types:
    # RoboCompDifferentialRobot.TMechParams

    ######################
    # From the RoboCompHumanCameraBody you can call this methods:
    # self.humancamerabody_proxy.newPeopleData(...)

    ######################
    # From the RoboCompHumanCameraBody you can use this types:
    # RoboCompHumanCameraBody.TImage
    # RoboCompHumanCameraBody.TGroundTruth
    # RoboCompHumanCameraBody.KeyPoint
    # RoboCompHumanCameraBody.Person
    # RoboCompHumanCameraBody.PeopleData

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
