#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2020 by YOUR NAME HERE
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

from genericworker import *
import numpy as np
import sys
import os
import cv2
import datetime

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
sys.path.append(os.path.join(os.getcwd(),"assets"))
print(sys.path)
import detection_rectangles
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 50
        self.timer.start(self.Period)
        self.im_width = 640
        self.im_height = 360
        self.detection_graph, self.sess = detection_rectangles.load_inference_graph()
        self.start_time = datetime.datetime.now()
        self.num_frames = 0

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        return True


    @QtCore.Slot()
    def compute(self):
        print('SpecificWorker.compute...')
        try:
            data = self.camerasimple_proxy.getImage()
            self.HandGestureClient_getHandGesture(data)
            return True
        except:
            print("Error taking camera feed. Make sure Camerasimple is up and running")



    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # getHandGesture
    #
    def HandGestureClient_getHandGesture(self, handImg):
        #
        # implementCODE
        #
        try:
            arr = np.fromstring(handImg.image, np.uint8)
            frame = np.reshape(arr, (handImg.height, handImg.width, handImg.depth))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = cv2.resize(frame, (self.im_width, self.im_height))
            relative_boxes, scores, classes = detection_rectangles.detect_objects(frame, self.detection_graph, self.sess)
            maxscore_idx = np.where(scores == scores.max())
            required_box = relative_boxes[maxscore_idx][0]
            print(required_box)
            box_relative2absolute = lambda box: (box[1] * self.im_width, box[3] * self.im_width, box[0] * self.im_height, box[2] * self.im_height)
            hand_box = box_relative2absolute(required_box)
            # cv2.imshow('Testing',frame)
            (left, right, top, bottom) = hand_box
            p1 = (int(left), int(top))
            p2 = (int(right), int(bottom))
            box_color = (204, 41, 0)
            cv2.rectangle(frame, p1, p2, box_color, 3, 1)
            self.num_frames+=1
            elapsed_time = (datetime.datetime.now() - self.start_time).total_seconds()
            fps = self.num_frames / elapsed_time
            print_text = "FPS : " + str(int(fps))
            text_color = (0,0,0)
            cv2.putText(frame, print_text, (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)
            cv2.imshow('Hand Gesture Client',cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        except Exception as e:
            print(e)
            print("Error processing input image")
        hand = HandType()
        return hand

    # ===================================================================
    # ===================================================================
