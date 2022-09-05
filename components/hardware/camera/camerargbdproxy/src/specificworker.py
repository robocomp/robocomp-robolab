#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2022 by YOUR NAME HERE
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
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import traceback
import time

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, params, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 33
        self.fps = 30
        self.camera_name = 'camera_top'

        # params
        try:
            self.camera_name = params["camera_name"]
            self.fps = int(params["fps"])
            self.Period = int(1000.0/self.fps)
            print("Config params:", params)
        except:
            print("Error reading config params")
            traceback.print_exc()

        if startup_check:
            self.startup_check()
        else:
            # Hz
            self.cont = 0
            self.last_time = time.time()
            self.fps = 0

            # camera read thread
            #self.read_queue = queue.Queue(1)
            #self.read_thread = Thread(target=self.get_rgb_thread, args=["camera_top"], name="read_queue")
            # self.read_thread.start()
            self.rgb_read = None
            self.rgb_write = None

            print("Component initialized")
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    # def setParams(self, params):
    #     try:
    #         self.camera_name = params["camera_name"]
    #         self.fps = params["fps"]
    #     except:
    #         print("Error reading config params")
    #         traceback.print_exc()
    #     return True

    @QtCore.Slot()
    def compute(self):
        try:
            self.rgb_read = self.camerargbdsimple_proxy.getImage(self.camera_name)
        except:
            print("Error communicating with CameraRGBDSimple")
            traceback.print_exc()
            sys.exit()

        self.rgb_read, self.rgb_write = self.rgb_write, self.rgb_read

        # FPS
        self.show_fps()

        return True

    #####################################################################################
    # def get_rgb_thread(self, camera_name: str):
    #     while True:
    #         try:
    #             rgb = self.camerargbdsimple_proxy.getImage(camera_name)
    #             self.image_captured_time = time.time()
    #             frame = np.frombuffer(rgb.image, dtype=np.uint8)
    #             frame = frame.reshape((rgb.height, rgb.width, 3))
    #             self.read_image_queue.put(frame)
    #         except:
    #             print("Error communicating with CameraRGBDSimple")
    #             traceback.print_exc()
    #             break

    def show_fps(self):
        if time.time() - self.last_time > 1:
            self.last_time = time.time()
            print("Freq: ", self.cont, "Hz. Waiting for image")
            self.cont = 0
        else:
            self.cont += 1
    def startup_check(self):
        print(f"Testing RoboCompCameraRGBDSimple.TImage from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TImage()
        print(f"Testing RoboCompCameraRGBDSimple.TDepth from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TDepth()
        print(f"Testing RoboCompCameraRGBDSimple.TRGBD from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TRGBD()
        QTimer.singleShot(200, QApplication.instance().quit)

        # =============== Methods for Component Implements ==================
        # ===================================================================

        #
        # IMPLEMENTATION of getAll method from CameraRGBDSimple interface
        #

    def CameraRGBDSimple_getAll(self, camera):
        ret = ifaces.RoboCompCameraRGBDSimple.TRGBD()
        #
        # write your CODE here
        #
        return ret
        #
        # IMPLEMENTATION of getDepth method from CameraRGBDSimple interface
        #

    def CameraRGBDSimple_getDepth(self, camera):
        ret = ifaces.RoboCompCameraRGBDSimple.TDepth()
        #
        # write your CODE here
        #
        return ret
        #
        # IMPLEMENTATION of getImage method from CameraRGBDSimple interface
        #

    def CameraRGBDSimple_getImage(self, camera):
        return self.rgb_read

    # ===================================================================
    # ===================================================================


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


