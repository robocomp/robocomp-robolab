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
from RoboCompBodyHandJointsDetector import *
import traceback
import cv2
import numpy as np
from image_utils import draw_pose
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
        self.timer.timeout.connect(self.compute)
        self.Period = 30
        self.timer.start(self.Period)
        self.cam_timer = Timer(fps=30)
        self.pose_timer = Timer(fps=30)
        self.print_timer = Timer(fps=0.5)
        self.capL = cv2.VideoCapture(0)

        self.max_num_images = 1
        self.list_frames = []

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
                self.list_frames.append(self.frameL)

        if self.pose_timer.isReady(now) and len(self.list_frames) > self.max_num_images:
            start_time = time.time()
            input = TImage()
            input.image = np.stack(self.list_frames, axis = 0)
            input.height, input.width, input.depth = self.list_frames[0].shape
            input.numImages = len(self.list_frames)
            store_frames = self.list_frames
            self.list_frames = []

            list_body = self.bodyhandjointsdetector_proxy.getBodyAndHand(input)


            # visualize
            if cam_ready:
                for i,ele in enumerate(list_body):
                    best_body = np.array(ele.keyPoints)
                    best_body = np.reshape(best_body, (61,2))
                    draw_pose(store_frames[i], best_body)
                    cv2.imshow("visual", store_frames[i])
                store_frames = []


        return True

    def startup_check(self):
        import time
        time.sleep(0.2)
        exit()




    ######################
    # From the RoboCompBodyHandJointsDetector you can call this methods:
    # self.bodyhandjointsdetector_proxy.getBodyAndHand(...)

    ######################
    # From the RoboCompBodyHandJointsDetector you can use this types:
    # RoboCompBodyHandJointsDetector.TImage
    # RoboCompBodyHandJointsDetector.TBody

    ######################
    # From the RoboCompCameraSimple you can call this methods:
    # self.camerasimple_proxy.getImage(...)

    ######################
    # From the RoboCompCameraSimple you can use this types:
    # RoboCompCameraSimple.TImage


