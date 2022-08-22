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
import numpy as np
import time
import math
import cv2
import torch
import torch.backends.cudnn as cudnn

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)
import mediapipe as mp
import queue


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 50
        if startup_check:
            self.startup_check()
        else:
            # Hz
            self.cont = 0
            self.last_time = time.time()

            # display
            self.display = False

            # pose
            self.mp_drawing = mp.solutions.drawing_utils
            self.mp_pose = mp.solutions.pose
            self.mediapipe_human_pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

            # queue
            self.input_queue = queue.Queue(1)
            self.output_queue = queue.Queue(1)

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        self.display = params["display"] == "true" or params["display"] == "True"
        return True


    @QtCore.Slot()
    def compute(self):
        self.detect_skeleton()
        return True

    ######################################################################
    def detect_skeleton(self):
        #ext_image = self.input_queue.get()
        try:
            rgbd = self.camerargbdsimple_proxy.getAll("camera_top")
        except:
            print("Error connecting to cameraRGBDSimple. Returning")
            return

        t0 = time.time()
        color = np.frombuffer(rgbd.image.image, dtype=np.uint8)
        color = color.reshape((rgbd.image.height, rgbd.image.width, 3))
        image = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        rows, cols, _ = image.shape
        image.flags.writeable = False
        t1 = time.time()
        pose_results = self.mediapipe_human_pose.process(image)
        t2 = time.time()

        if self.display:
            self.mp_drawing.draw_landmarks(color, pose_results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
            cv2.imshow("Jetson", color)
            cv2.waitKey(1)  # 1 millisecond

        # fill interface data
        objects = []
        #self.output_queue.put(objects)    # synchronize with interface
        t3 = time.time()
        print(f'Total {(1E3 * (t3 - t0)):.1f}ms, Inference {(1E3 * (t2 - t1)):.1f}ms')

    ######################################################################
    def startup_check(self):
        print(f"Testing RoboCompCameraRGBDSimple.TImage from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TImage()
        print(f"Testing RoboCompCameraRGBDSimple.TDepth from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TDepth()
        print(f"Testing RoboCompCameraRGBDSimple.TRGBD from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TRGBD()
        print(f"Testing RoboCompHumanCameraBodyPub.TImage from ifaces.RoboCompHumanCameraBodyPub")
        test = ifaces.RoboCompHumanCameraBodyPub.TImage()
        print(f"Testing RoboCompHumanCameraBodyPub.TGroundTruth from ifaces.RoboCompHumanCameraBodyPub")
        test = ifaces.RoboCompHumanCameraBodyPub.TGroundTruth()
        print(f"Testing RoboCompHumanCameraBodyPub.KeyPoint from ifaces.RoboCompHumanCameraBodyPub")
        test = ifaces.RoboCompHumanCameraBodyPub.KeyPoint()
        print(f"Testing RoboCompHumanCameraBodyPub.Person from ifaces.RoboCompHumanCameraBodyPub")
        test = ifaces.RoboCompHumanCameraBodyPub.Person()
        print(f"Testing RoboCompHumanCameraBodyPub.PeopleData from ifaces.RoboCompHumanCameraBodyPub")
        test = ifaces.RoboCompHumanCameraBodyPub.PeopleData()
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
        QTimer.singleShot(200, QApplication.instance().quit)


    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of newPeopleData method from HumanCameraBody interface
    #
    def HumanCameraBody_newPeopleData(self):
        #return self.output_queue.get()
        return ifaces.RoboCompHumanCameraBody.PeopleData()

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

    ######################
    # From the RoboCompHumanCameraBodyPub you can publish calling this methods:
    # self.humancamerabodypub_proxy.newPeopleData(...)

    ######################
    # From the RoboCompHumanCameraBodyPub you can use this types:
    # RoboCompHumanCameraBodyPub.TImage
    # RoboCompHumanCameraBodyPub.TGroundTruth
    # RoboCompHumanCameraBodyPub.KeyPoint
    # RoboCompHumanCameraBodyPub.Person
    # RoboCompHumanCameraBodyPub.PeopleData

    ######################
    # From the RoboCompHumanCameraBody you can use this types:
    # RoboCompHumanCameraBody.TImage
    # RoboCompHumanCameraBody.TGroundTruth
    # RoboCompHumanCameraBody.KeyPoint
    # RoboCompHumanCameraBody.Person
    # RoboCompHumanCameraBody.PeopleData


