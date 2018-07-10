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

import numpy as np
from PySide import QtGui, QtCore
from genericworker import *
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
        self.timer.start(self.Period)
        self.state = "None"
        self.expected_hands = 0
        self.hands = []
        self.font = cv2.FONT_HERSHEY_SIMPLEX


    def setParams(self, params):
        #try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        #except:
        #	traceback.print_exc()
        #	print "Error reading config params"
        return True

    @QtCore.Slot()
    def compute(self):
        start = time.time()
        try:
            image = self.camerasimple_proxy.getImage()
        except Ice.Exception, e:
            traceback.print_exc()
            print e
        frame = np.fromstring(image.image, dtype=np.uint8)
        frame = frame.reshape(image.width,image.height, image.depth)
        to_show = frame.copy()
        if self.state == "None":
            try:
                current_hand_count = self.handdetection_proxy.getHandsCount()

                if current_hand_count < 1:
                    try:
                        search_roi_class = TRoi()
                        search_roi_class.y = 480 / 2 - 100
                        search_roi_class.x = 640 / 2 - 100
                        search_roi_class.w = 200
                        search_roi_class.h =200
                        search_roi =(search_roi_class.x, search_roi_class.y, search_roi_class.h, search_roi_class.w)

                        to_show = self.draw_initial_masked_frame(to_show, search_roi)
                        self.expected_hands = self.handdetection_proxy.addNewHand(1, search_roi_class)
                    except Ice.Exception, e:
                        traceback.print_exc()
                        print e
                elif current_hand_count >=  self.expected_hands:
                    self.state = "tracking"
            except Ice.Exception, e:
                traceback.print_exc()
                print e
        elif self.state == "tracking":
            try:
                self.hands = self.handdetection_proxy.getHands()
                for hand in self.hands:
                    to_show = self.draw_hand_overlay(to_show,hand)
            except Ice.Exception, e:
                traceback.print_exc()
                print e
        cv2.imshow("tvGame visualization", to_show)
        cv2.waitKey(1)
        print "SpecificWorker.compute... in state %s with %d hands" % (self.state, len(self.hands))

        return True

    def draw_initial_masked_frame(self, frame, search_roi):
        masked_frame = np.zeros(frame.shape, dtype="uint8")
        masked_frame[::] = 255
        template_x, template_y, template_w, template_h = search_roi
        cv2.putText(masked_frame, "PLEASE PUT YOUR HAND HERE", (template_x - 100, template_y + template_h + 10),
                    self.font, 1, [0, 0, 0], 2)

        masked_frame = cv2.rectangle(masked_frame, (template_x, template_y),
                                     (template_x + template_w, template_y + template_h), [0, 0, 0])
        alpha = 0.7
        beta = (1.0 - alpha)
        masked_frame = cv2.addWeighted(frame, alpha, masked_frame, beta, 0.0)
        return masked_frame


    def draw_hand_overlay(self, frame, hand):
        if hand.detected:
            for finger_number, fingertip in enumerate(hand.fingertips):
                cv2.circle(frame, tuple(fingertip), 10, [255, 100, 255], 3)
                cv2.putText(frame, 'finger' + str(finger_number), tuple(fingertip), self.font, 0.5,
                            (255, 255, 255),
                            1)
            for defect in hand.intertips:
                cv2.circle(frame, tuple(defect), 8, [211, 84, 0], -1)
        # self.draw_contour_features(frame, hand.contour)
        # x, y, w, h = hand.bounding_rect
        # cv2.putText(frame, (str(w)), (x + w, y), self.font, 0.3, [255, 255, 255], 1)
        # cv2.putText(frame, (str(h)), (x + w, y + h), self.font, 0.3, [255, 255, 255], 1)
        # cv2.putText(frame, (str(w * h)), (x + w / 2, y + h / 2), self.font, 0.3, [255, 255, 255], 1)
        # cv2.putText(frame, (str(x)+", "+str(y)), (x-10, y-10), self.font, 0.3, [255, 255, 255], 1)
        # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        if hand.detected or hand.tracked:
            cv2.drawContours(frame, [np.array(hand.contour, dtype=int)], -1, (255, 255, 255), 2)

        points = np.array(hand.positions)
        cv2.polylines(img=frame, pts=np.int32([points]), isClosed=False, color=(255,0,200))
        tail_length = 15
        if len(points) > tail_length:
            for i in np.arange(1, tail_length):
                ci = len(points) - tail_length + i
                thickness = int((float(i) / tail_length) * 13) + 1
                cv2.line(frame, tuple(points[ci - 1]), tuple(points[ci]), (0, 0, 255), thickness)

        if hand.centerMass is not None:
            # Draw center mass
            cv2.circle(frame, tuple(hand.centerMass), 7, [100, 0, 255], 2)
            cv2.putText(frame, 'Center', tuple(hand.centerMass), self.font, 0.5, (255, 255, 255), 1)

        hand_string = "hand %d %s: D=%s|T=%s|L=%s" % (
        hand.id, str(hand.centerMass), str(hand.detected), str(hand.tracked), str(hand.truthValue))
        cv2.putText(frame, hand_string, (10, 30 + 15 * int(hand.id)), self.font, 0.5, (255, 255, 255), 1)
        return frame
