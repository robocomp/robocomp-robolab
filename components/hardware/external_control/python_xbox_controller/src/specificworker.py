#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2025 by YOUR NAME HERE
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

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from rich.console import Console
from genericworker import *
import interfaces as ifaces
import pygame
import numpy as np

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]
        if startup_check:
            self.startup_check()
        else:
            # Initialize Pygame and the joystick
            pygame.init()
            pygame.joystick.init()

            # Connect to the first joystick
            if pygame.joystick.get_count() == 0:
                print("No joystick found.")
                exit()

            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

            self.x_max_vel = 750
            self.y_max_vel = 400
            self.angle_max_vel = 2

            self.past_values = np.zeros(3)
            self.stop_counter = 0
            self.old_button = np.array([0,0,0])
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
        # Handle events to keep pygame in sync with the system
        pygame.event.pump()
        # Get odometry values from joystick
        odom_values = self.get_odom_with_joystick(np.array([self.joystick.get_axis(1),
                                           self.joystick.get_axis(0),
                                           self.joystick.get_axis(3)]))
        button_values = np.array([self.joystick.get_button(1), self.joystick.get_button(3), self.joystick.get_button(2)])
        # if hoystick values are 0, 0, 0 for 5 times, return
        if np.all(odom_values == 0) and np.array_equal(self.old_button, button_values) :
            self.stop_counter += 1
            if self.stop_counter > 4:
                self.Period = 100
                return
            
        else:
            if self.stop_counter > 4:
                self.Period = 16
                self.stop_counter = 0
                self.old_button = button_values

        print(odom_values, button_values)
        self.publish_odom(odom_values, button_values)

    def get_odom_with_joystick(self, joystick_values):
        # Read the left joystick axes (0 is horizontal, 1 is vertical)
        axis_x = joystick_values[0] if abs(joystick_values[0]) > 0.1 else 0 # Left stick horizontal movement
        axis_y = joystick_values[1] if abs(joystick_values[1]) > 0.1 else 0 # Left stick vertical movement

        # Assign the axes to the robot's movement
        x_odom = -axis_x * self.x_max_vel
        y_odom = -axis_y * self.y_max_vel

        # Read the right joystick axis (3 is horizontal movement)
        right_axis_x = joystick_values[2] if abs(joystick_values[2]) > 0.1 else 0 # Right stick horizontal movement

        # Assign the axis to the robot's rotation
        angle_odom = -right_axis_x * self.angle_max_vel

        return np.array([x_odom, y_odom ,angle_odom])

    def publish_odom(self, odom_values, button_values):
        axis_data = ifaces.RoboCompJoystickAdapter.AxisList([ifaces.RoboCompJoystickAdapter.AxisParams(name="side", value=float(odom_values[1])),
                                                              ifaces.RoboCompJoystickAdapter.AxisParams(name="advance", value=float(odom_values[0])),
                                                              ifaces.RoboCompJoystickAdapter.AxisParams(name="rotate", value=float(odom_values[2]))])
        button_data = ifaces.RoboCompJoystickAdapter.ButtonsList([ifaces.RoboCompJoystickAdapter.ButtonParams(name="block", step=button_values[0]),
                                                                 ifaces.RoboCompJoystickAdapter.ButtonParams(name="stop", step=button_values[1]),
                                                                 ifaces.RoboCompJoystickAdapter.ButtonParams(name="joystica_control", step=button_values[2])])
        self.joystickadapter_proxy.sendData(ifaces.RoboCompJoystickAdapter.TData(axes=axis_data, buttons=button_data))

    def startup_check(self):
        print(f"Testing RoboCompJoystickAdapter.AxisParams from ifaces.RoboCompJoystickAdapter")
        test = ifaces.RoboCompJoystickAdapter.AxisParams()
        print(f"Testing RoboCompJoystickAdapter.ButtonParams from ifaces.RoboCompJoystickAdapter")
        test = ifaces.RoboCompJoystickAdapter.ButtonParams()
        print(f"Testing RoboCompJoystickAdapter.TData from ifaces.RoboCompJoystickAdapter")
        test = ifaces.RoboCompJoystickAdapter.TData()
        QTimer.singleShot(200, QApplication.instance().quit)




    ######################
    # From the RoboCompJoystickAdapter you can publish calling this methods:
    # RoboCompJoystickAdapter.void self.joystickadapter_proxy.sendData(TData data)

    ######################
    # From the RoboCompJoystickAdapter you can use this types:
    # ifaces.RoboCompJoystickAdapter.AxisParams
    # ifaces.RoboCompJoystickAdapter.ButtonParams
    # ifaces.RoboCompJoystickAdapter.TData


