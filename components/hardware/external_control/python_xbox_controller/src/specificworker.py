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

            # Axis and button mappings are read from the config file
            self.axes, self.buttons = self.load_joystick_mapping(configData)

            self.past_values = np.zeros(len(self.axes))
            self.stop_counter = 0
            self.old_button = np.zeros(len(self.buttons), dtype=int)
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

    def load_joystick_mapping(self, configData):
        """Read axis and button mappings from the config file.

        Expected entries (see etc/config), one CSV string per axis/button:
            joystickUniversal.NumAxes    = N
            joystickUniversal.Axis_i     = name, axis_index, min, max, inverted, dead_zone
            joystickUniversal.NumButtons = M
            joystickUniversal.Button_i   = name, button_index, step
        """
        jc = configData["joystickUniversal"]

        axes = []
        for i in range(int(jc["NumAxes"])):
            name, axis, mn, mx, inverted, dead_zone = [s.strip() for s in str(jc[f"Axis_{i}"]).split(",")]
            axes.append({
                "name": name,
                "axis": int(axis),
                "min": float(mn),
                "max": float(mx),
                "inverted": inverted.lower() == "true",
                "dead_zone": float(dead_zone),
            })

        buttons = []
        for i in range(int(jc["NumButtons"])):
            name, index, step = [s.strip() for s in str(jc[f"Button_{i}"]).split(",")]
            buttons.append({
                "name": name,
                "index": int(index),
                "step": int(step),
            })

        return axes, buttons


    @QtCore.Slot()
    def compute(self):
        # Handle events to keep pygame in sync with the system
        pygame.event.pump()
        # Get odometry values and button states from the joystick, following the config mapping
        odom_values = np.array([self.read_axis(a) for a in self.axes])
        button_values = np.array([self.read_button(b) for b in self.buttons])
        # if joystick values are all 0 and buttons unchanged for 5 times, slow down and return
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

    def read_axis(self, axis):
        # Read a single joystick axis, apply dead zone, scale to [min, max] and invert if requested
        raw = self.joystick.get_axis(axis["axis"])
        if abs(raw) <= axis["dead_zone"]:
            return 0.0
        value = raw * (axis["max"] if raw >= 0 else -axis["min"])
        return -value if axis["inverted"] else value

    def read_button(self, button):
        # Button state (0/1) scaled by the configured step
        return int(self.joystick.get_button(button["index"])) * button["step"]

    def publish_odom(self, odom_values, button_values):
        axis_data = ifaces.RoboCompJoystickAdapter.AxisList(
            [ifaces.RoboCompJoystickAdapter.AxisParams(name=a["name"], value=float(v))
             for a, v in zip(self.axes, odom_values)])
        button_data = ifaces.RoboCompJoystickAdapter.ButtonsList(
            [ifaces.RoboCompJoystickAdapter.ButtonParams(name=b["name"], step=int(v))
             for b, v in zip(self.buttons, button_values)])
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


