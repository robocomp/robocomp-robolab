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
import time
import copy

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
sys.path.append(os.path.join(os.getcwd(),"assets"))

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 20
        self.timer.start(self.Period)
        self.method = 'Mediapipe'
        # Method = 'Mediapipe' for hand detection using Mediapipe Deep Learning Models
        # Method = 'SSD' for hand detection using SSD + MobileNet
        if(self.method=='SSD'):
            try:
                global detection_ssd
                import detection_ssd
                self.detection_graph, self.sess = detection_ssd.load_inference_graph()
            except:
                print("Error Loading Model. Ensure that models are downloaded and placed in correct directory")
        elif(self.method=='Mediapipe'):
            try:
                global detection_mediapipe
                import detection_mediapipe
            except:
                print("Error Loading Model. Ensure that models are downloaded and placed in correct directory")

        # Gesture display configurations
        self.nogesture_color = (204,0,0)
        self.gesture_color = (0,0,255)

        # Bounding Box display configurations
        self.bbox_color = (204, 41, 0)
        self.bbox_point_color = (204, 41, 0)
        self.keypoint_color = (0, 255, 0)
        self.conn_color = (255, 0, 0)
        self.thickness = 2
        self.fps_text_color = (0,0,0)
        # storing program runtime and processed frames for calculating FPS
        self.start_time = 0
        # Gesture Recognition Labels (ASL)
        self.gesture_labels = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", 
                                "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "del", "space"]


        print("\n\nPlease specify ASL alphabets set you want to classify from (Space Seperated, Upper Case). Press Enter to use all classes")

        if(self.method == 'Mediapipe'):
            labels = input()
            if(labels != ""):
                self.gesture_labels = labels.split()

            print("Detecting for following classes")
            print(self.gesture_labels)

            self.handgesture_proxy.setClasses(self.gesture_labels)

        print('Component Started')
        
    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        return True

    @QtCore.Slot()
    def compute(self):
        self.start_time = time.time()
        try:
            data = self.camerasimple_proxy.getImage()
        except:
            print("Error taking camera feed. Make sure Camerasimple is up and running")
            return False

        handData = self.HandGestureClient_getHandGesture(data)
        self.display(data,handData)
        return True

    def display(self,handImg,handData):
        elapsed_time = time.time() - self.start_time
        arr = np.fromstring(handImg.image, np.uint8)
        frame = np.reshape(arr, (handImg.height, handImg.width, handImg.depth))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if(self.method=='SSD'):
            frame = cv2.resize(frame, (self.im_width, self.im_height))

        ## Add Hand Bounding Box in image frame
        bbox = handData.boundingbox
        if(bbox is not None):
            for i in range(4):
                x0, y0 = bbox[i]
                x1, y1 = bbox[(i+1)%4]
                cv2.line(frame, (int(x0), int(y0)), (int(x1), int(y1)),
                                    self.bbox_color, self.thickness)

        ## Add detected keypoints in image frame

        detected_keypoints = handData.keypoint
        if(detected_keypoints is not None):
            connections = None
            if(self.method=='SSD'):
                connections = [
                                (0,1),(1,2),(2,3),(3,4),(0,5),
                                (5,6),(6,7),(7,8),(0,9),(9,10),
                                (10,11),(11,12),(0,13),(13,14),
                                (14,15),(15,16),(0,17),(17,18),
                                (18,19),(19,20)
                            ]
            else:
                connections = [
                            (0,1),(1,2),(2,3),(3,4),
                            (5,6),(6,7),(7,8),(9,10),
                            (10,11),(11,12),(13,14),
                            (14,15),(15,16),(17,18),(18,19),
                            (19, 20),(0,5),(5,9),(9,13),
                            (13,17),(0,17)
                        ]

            for point in detected_keypoints:
                x = point[0]
                y = point[1]
                cv2.circle(frame, (int(x), int(y)), self.thickness*2, self.keypoint_color, self.thickness)

            for connection in connections:
                x0 = detected_keypoints[connection[0]][0]
                y0 = detected_keypoints[connection[0]][1]
                x1 = detected_keypoints[connection[1]][0]
                y1 = detected_keypoints[connection[1]][1]

                cv2.line(frame, (int(x0), int(y0)), (int(x1), int(y1)), self.conn_color, self.thickness)

        ## Add Recognised Gesture in frame
        if(handData.gesture == "None"):
            print_text = "No Hand Detected"
            cv2.putText(frame, print_text, (320,50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.nogesture_color, 2)        
        else:
            x,y = bbox[0]
            print_text = handData.gesture
            cv2.putText(frame, print_text, (int(x),int(y)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, self.gesture_color, 2)              


        ## insert FPS in frame
        fps = 1.0 / elapsed_time
        print_text = "FPS : " + str(int(fps))
        cv2.putText(frame, print_text, (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.fps_text_color, 2)

        ## Display image using OpenCV
        cv2.imshow('Hand Gesture Client',cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

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
            frame_cp = copy.deepcopy(frame)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            detected_keypoints = None
            if(self.method=='SSD'):
                ## Resizing to required size
                self.im_width = handImg.width
                self.im_height = handImg.height
                ## Detecting boxes with hand in image
                relative_boxes, scores, classes = detection_ssd.detect_objects(
                                        frame, self.detection_graph, self.sess)

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

                sendBbox = [left, right, top, bottom]
            elif(self.method=='Mediapipe'):
                bbox, detected_keypoints, raw_keypoints = detection_mediapipe.hand_detector(frame)
                if(bbox is not None):
                    min_idx = np.amin(bbox,axis=0)
                    max_idx = np.amax(bbox, axis=0)
                    sendBbox = [min_idx[0], max_idx[0], min_idx[1], max_idx[1]]
                    print('Hand Bounding Box and Keypoints detected')
                else:
                    print("No hand detected")
            else:
                print("Error! Please enter valid detection method number")
                bbox = None


        except Exception as e:
            bbox = None
            print(e)
            print("Error processing input image")

        if(self.method=='SSD' and bbox is not None):
            ## Detecting Keypoint using Openpose (using HandKeypoint Component)
            sendHandImage = RoboCompHandKeypoint.TImage()
            sendHandImage.image = frame_cp
            sendHandImage.height, sendHandImage.width, sendHandImage.depth = frame_cp.shape
            detected_keypoints = self.handkeypoint_proxy.getKeypoints(sendHandImage, list(sendBbox))
            if(detected_keypoints is None):
                print("Keypoints not detected")
            else:
                print("Keypoint Detected")

        gesture = None
        if(self.method=='Mediapipe' and bbox is not None and detected_keypoints is not None):
            sendKeypoints = detected_keypoints
            sendKeys = []
            for key in raw_keypoints:
                sendKeys.append(list(key))
            gesture = self.handgesture_proxy.getHandGesture(sendKeys)
            print('Gesture Detected')

        hand = HandType()
        hand.boundingbox = bbox
        hand.keypoint = detected_keypoints
        if(gesture is not None):
            hand.gesture = gesture
        else:
            hand.gesture = "None"
        return hand
    # ===================================================================
    # ===================================================================
