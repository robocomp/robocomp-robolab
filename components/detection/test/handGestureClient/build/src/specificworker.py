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
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 20
        self.timer.start(self.Period)
        self.method = 2
        # Method = 1 for hand detection using SSD + MobileNet
        # Method = 2 for hand detection using Mediapipe Deep Learning Models
        if(self.method==1):
            try:
                global detection_ssd
                import detection_ssd
                self.im_width = 640 # Change to change image with while processing
                self.im_height = 360 # Change to change height with while processing
                self.detection_graph, self.sess = detection_ssd.load_inference_graph()
            except:
                print("Error Loading Model. Ensure that models are downloaded \
                                            and placed in correct directory")
        elif(self.method==2):
            try:
                global detection_mediapipe
                import detection_mediapipe
            except:
                print("Error Loading Model. Ensure that models are downloaded \
                            and placed in correct directory")


        # Bounding Box display configurations
        self.bbox_color = (204, 41, 0)
        self.bbox_point_color = (204, 41, 0)
        self.thickness = 2
        self.fps_text_color = (0,0,0)
        # storing program runtime and processed frames for calculating FPS
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
        except:
            print("Error taking camera feed. Make sure Camerasimple is up and running")
            return False
        self.HandGestureClient_getHandGesture(data)
        return True

        return True



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
            ## Rearranging to form numpy matrix
            arr = np.fromstring(handImg.image, np.uint8)
            frame = np.reshape(arr, (handImg.height, handImg.width, handImg.depth))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            if(self.method==1):
                ## Resizing to required size
                frame = cv2.resize(frame, (self.im_width, self.im_height))

                ## Detecting boxes with hand in image
                relative_boxes, scores, classes = detection_ssd.detect_objects(frame, self.detection_graph, self.sess)

                ## Currenty, only one hand with maximum score is considered
                maxscore_idx = np.where(scores == scores.max())
                required_box = relative_boxes[maxscore_idx][0]
                print("Bounding Box Found")

                box_relative2absolute = lambda box: (box[1] * self.im_width, box[3] * self.im_width, box[0] * self.im_height, box[2] * self.im_height)
                hand_box = box_relative2absolute(required_box)

                ## insert bounding box in image
                (left, right, top, bottom) = hand_box

                bbox = [
                    (int(left), int(bottom)),
                    (int(left), int(top)),
                    (int(right), int(top)),
                    (int(right), int(bottom)),
                ]
            elif(self.method==2):
                bbox = detection_mediapipe.hand_detector(frame)
                if(bbox is None):
                    print("No hand detected")
            else:
                print("Error! Please enter valid detection method number")
                bbox = None

            if(bbox is not None):
                print('Bounding Box Coordinates are:')
                print(bbox)
                for pt in bbox:
                	x, y = pt
                	cv2.circle(frame, (int(x), int(y)), self.thickness*2,
                                    self.bbox_point_color, self.thickness)
                for i in range(4):
                    x0, y0 = bbox[i]
                    x1, y1 = bbox[(i+1)%4]
                    cv2.line(frame, (int(x0), int(y0)), (int(x1), int(y1)),
                                        self.bbox_color, self.thickness)

            # insert FPS in image
            self.num_frames+=1
            elapsed_time = (datetime.datetime.now() - self.start_time).total_seconds()
            fps = self.num_frames / elapsed_time
            print_text = "FPS : " + str(int(fps))
            cv2.putText(frame, print_text, (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.fps_text_color, 2)

            ## Display image using OpenCV
            cv2.imshow('Hand Gesture Client',cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))


        except Exception as e:
            print(e)
            print("Error processing input image")
        hand = HandType()
        return hand
    # ===================================================================
    # ===================================================================


