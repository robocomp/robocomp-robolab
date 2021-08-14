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
import cv2
import pickle as pkl

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


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
        self.timer.timeout.connect(self.compute)
        self.Period = 30
        self.timer.start(self.Period)
        # self.visualizer = Visualizer()
        self.cam_timer = Timer(fps=30)
        self.inference_timer = Timer(fps=10)
        self.capL = cv2.VideoCapture(0)

        self.max_num_images = 64
        self.list_frames = []

        self.wlasl_class = pkl.load( open("src/wlasl_name.pkl", "rb" ) )

        self.last_class = ""
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.classes = {}

    def __del__(self):
        console.print('SpecificWorker destructor')

    def setParams(self, params):
        return True

    @QtCore.Slot()
    def compute(self):
        now = current_milli_time()
        cam_ready = self.cam_timer.isReady(now)
        if cam_ready:
            retL, self.frameL = self.capL.read()
            if self.frameL is not None:
                if self.last_class != "":
                    cv2.putText(self.frameL, self.last_class, (20,20), self.font, 10, (0,0,0), 2)
                cv2.imshow("visual", self.frameL)
                self.list_frames.append(self.frameL)

        if self.inference_timer.isReady(now) and len(self.list_frames) >= self.max_num_images:
            input = TVideo()

            input.images = np.stack(self.list_frames, axis=0)
            input.height, input.width, input.depth = self.list_frames[0].shape
            input.numFrames = len(self.list_frames)

            self.list_frames = self.list_frames[len(self.list_frames)//2:]
            output = self.imagebasedgesturerecognition_proxy.getGesture(input)

            if output.gestureProb[0] > 0.25:
                print("top 5 actions")
                action = []
                for ele in output.gestureIndex:
                    action.append(self.wlasl_class[int(ele)])
                print(action)
                print(output.gestureProb)
                self.last_class = action[0]

                if action[0] not in self.classes:
                    self.classes[action[0]] = 1
                else:
                    self.classes[action[0]] +=1
                print(self.classes)
            else:
                print("nothing")
                self.last_class = ""

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
