#
# Copyright (C) 2018 by YOUR NAME HERE
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

import sys, os, traceback, time

from PySide import QtGui, QtCore
from genericworker import *
from libs.HandDetection.HandDetection import HandDetector
import numpy as np
import cv2


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 20
        self.hand_detector = HandDetector(-1)
        self.hand_detector.set_mask_mode("depth")
        self.timer.start(self.Period)
        self.state = "None"
        self.new_hand_roi = None
        self.expected_hands = 0
        self.hand_detector.debug = True



    def setParams(self, params):
        #try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        #except:
        #	traceback.print_exc()
        #	print "Error reading config params"
        return True

    @QtCore.Slot()
    def compute(self):
        try:
            # image = self.camerasimple_proxy.getImage()
            # frame = np.fromstring(image.image, dtype=np.uint8)
            # frame = frame.reshape(image.width, image.height, image.depth)

            color, depth, _, _ = self.rgbd_proxy.getData()
            frame = np.fromstring(color, dtype=np.uint8)
            frame = frame.reshape(480, 640, 3)
            depth = np.array(depth,  dtype=np.uint8)
            depth = depth.reshape(480, 640, 1)
            print "showing depth"
            self.hand_detector.set_depth_mask(depth)
            cv2.imshow("depth specific", depth)
            cv2.imshow("color", frame)


        except Ice.Exception, e:
            traceback.print_exc()
            print e
            return False

        if self.state == "add_new_hand":
            search_roi = (self.new_hand_roi.x, self.new_hand_roi.y, self.new_hand_roi.w, self.new_hand_roi.h)
            self.hand_detector.add_hand2(frame, search_roi)
            if len(self.hand_detector.hands) >= self.expected_hands:
                self.state = "tracking"
                self.new_hand_roi = None
        elif self.state == "tracking":
            self.hand_detector.update_detection_and_tracking(frame)
        print "Compute in state %s with %d hands" % (self.state, len(self.hand_detector.hands))
        return True

        #
        # addNewHand
        #

    def addNewHand(self, expected_hands, new_hand_roi):
        self.new_hand_roi = new_hand_roi
        self.expected_hands = expected_hands
        self.state = "add_new_hand"
        return self.expected_hands


    def getHands(self):
        ret = []
        for detected_hand in self.hand_detector.hands:
            new_hand = Hand()
            new_hand.id = detected_hand.id
            new_hand.score = detected_hand.truth_value
            new_hand.fingertips = detected_hand.fingertips
            new_hand.intertips = detected_hand.intertips
            new_hand.positions = detected_hand.position_history
            print detected_hand.contour
            new_hand.contour = detected_hand.contour
            new_hand.centerMass = detected_hand.center_of_mass
            new_hand.truthValue = detected_hand.truth_value
            new_hand.detected = detected_hand.detected
            new_hand.tracked = detected_hand.tracked
            ret.append(new_hand)
        return ret


    def getHandsCount(self):
        return len(self.hand_detector.hands)





