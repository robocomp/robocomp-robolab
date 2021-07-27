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

from rich.console import Console
from genericworker import *
from RoboCompImageBasedGestureRecognition import *
import traceback
import numpy as np
import time

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class Timer:
    def __init__(self, fps):
        self.fps = fps
        self.ms = 1000 / fps
        self.tick = None

    # tock - current time in milliseconds
    def isReady(self, tock):
        if self.tick is None or tock - self.tick > self.ms:
            self.tick = tock
            return True
        return False


current_milli_time = lambda: int(round(time.time() * 1000))


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 2000
        self.timer.start(self.Period)
        # self.visualizer = Visualizer()
        self.cam_timer = Timer(fps=30)
        self.inference_timer = Timer(fps=10)
        self.print_timer = Timer(fps=0.5)

    def __del__(self):
        console.print('SpecificWorker destructor')

    def setParams(self, params):
        return True

    def compute(self):
        now = current_milli_time()
        cam_ready = self.cam_timer.isReady(now)
        if cam_ready:
            try:
                self.camera_image = self.camerasimple_proxy.getImage()
                arr = np.fromstring(self.camera_image.image, np.uint8)
                self.img_restored = np.reshape(arr, (
                    self.camera_image.width, self.camera_image.height, self.camera_image.depth))
            except Ice.Exception as e:
                traceback.print_exc()
                print(e)

        if self.inference_timer.isReady(now):
            #TODO: preprocess to TVideo type
            labels, probs = self.imagebasedgesturerecognition_proxy.getGesture(
                self.img_restored.data, [self.camera_image.width, self.camera_image.height, self.camera_image.depth])

            print(labels)

        return True

    def startup_check(self):
        import time
        time.sleep(0.2)
        exit()

    ######################
    # From the RoboCompCameraSimple you can call this methods:
    # self.camerasimple_proxy.getImage(...)

    ######################
    # From the RoboCompCameraSimple you can use this types:
    # RoboCompCameraSimple.TImage

    ######################
    # From the RoboCompImageBasedGestureRecognition you can call this methods:
    # self.imagebasedgesturerecognition_proxy.getGesture(...)

    ######################
    # From the RoboCompImageBasedGestureRecognition you can use this types:
    # RoboCompImageBasedGestureRecognition.TVideo
    # RoboCompImageBasedGestureRecognition.GestureResult