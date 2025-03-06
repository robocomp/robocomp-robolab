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
import cv2
import numpy as np

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 50
        self.battery_timer = QTimer()
        # connect the timer to the updateBatteryStatus function
        self.battery_timer.timeout.connect(self.updateBatteryStatus)

        self.battery_timer.start(5000)
        if startup_check:
            self.startup_check()
        else:
            self.show()
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):

        return True


    @QtCore.Slot()
    def compute(self):
        #ret, frame = self.cap.read()
        try:
            res = self.camerargbdsimple_proxy.getImage("")
            color = res.image
            depth = res.depth
            if res.compressed:
                # Convert the compressed image buffer to a numpy array
                compressed_image_np = np.frombuffer(color, dtype=np.uint8)
                # Decode the compressed image
                cvcolor = cv2.imdecode(compressed_image_np, cv2.IMREAD_COLOR)
                #print(compressed_image_np.size, cvcolor.size)
                print(compressed_image_np.size, cvcolor.size)

            #cvdepth = np.frombuffer(depth.depth, dtype=np.float32).reshape(depth.height, depth.width)
            #cvcolor = np.frombuffer(, dtype=np.uint8).reshape(color.height, color.width, 3)
            #cvcolor = np.frombuffer(color.image, dtype=np.uint8).reshape(color.height, color.width, 3)

        except Exception as e:
            print(e)
            return False

        frame_rgb = cv2.cvtColor(cvcolor, cv2.COLOR_BGR2RGB)
        height, width, channels = frame_rgb.shape
        bytes_per_line = channels * width
        q_image = QImage(frame_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)
        self.ui.video_label.setPixmap(QPixmap.fromImage(q_image))

        return True

    def updateBatteryStatus(self):
        """ Updates the battery level on the GUI and in the terminal """
        # Get battery status
        try:
            battery = self.batterystatus_proxy.getBatteryState()
            print("Battery: ", battery.percentage)
            if self.ui.progressBattery:
                self.ui.progressBattery.setValue(int(battery.percentage))  # Update GUI label
            print(f"Battery Level: {battery.percentage}%")  # Print battery level to the terminal
        except Exception as e:
            print("Error retrieving battery status:", e)

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)





