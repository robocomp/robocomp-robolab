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
        self.Period = 2000
        self.timer.start(self.Period)
        self.state = "None"
        self.expected_hands = 0
        self.hands = []


    def setParams(self, params):
        #try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        #except:
        #	traceback.print_exc()
        #	print "Error reading config params"
        return True

    @QtCore.Slot()
    def compute(self):
        print 'SpecificWorker.compute...'

        start = time.time()
        if self.state == "None":
            try:
                current_hand_count = self.handdetection_proxy.getHandsCount()

                if current_hand_count < 1:
                    try:
                        search_roi = TRoi()
                        search_roi.y = 480 / 2 - 100
                        search_roi.x = 640 / 2 - 100
                        search_roi.w = 200
                        search_roi.h =200
                        self.expected_hands = self.handdetection_proxy.addNewHand(1, search_roi)
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
                print self.hands
            except Ice.Exception, e:
                traceback.print_exc()
                print e
        print "Compute in state %s with %d hands" % (self.state, len(self.hands))

        return True

