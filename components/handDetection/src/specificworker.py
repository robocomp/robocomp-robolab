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
from libs.HandDetection.HandDetection import HandDetector, Hand, np


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
        self.hand_detector = HandDetector(-1)
        self.timer.start(self.Period)



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
        #computeCODE
        #try:
        #	self.differentialrobot_proxy.setSpeedBase(100, 0)
        #except Ice.Exception, e:
        #	traceback.print_exc()
        #	print e

        # The API of python-innermodel is not exactly the same as the C++ version
        # self.innermodel.updateTransformValues("head_rot_tilt_pose", 0, 0, 0, 1.3, 0, 0)
        # z = librobocomp_qmat.QVec(3,0)
        # r = self.innermodel.transform("rgbd", z, "laser")
        # r.printvector("d")
        # print r[0], r[1], r[2]

        return True


    #
    # processImage
    #
    def processImage(self, img):
        ret = []
        new_hand = RoboCompHandDetection.Hand()


        frame = np.zeros((img.height,img.width,img.depth), dtype=np.int8)
        print len(img.image)
        frame = np.fromstring(img.image, np.uint8)

        frame = frame.reshape((img.height,img.width,img.depth))
        if len(self.hand_detector.hands) < 1:
            self.hand_detector.add_hand2(frame)
        else:
            self.hand_detector.update_detection_and_tracking(frame)
            for detected_hand in self.hand_detector.hands:
                new_hand = Hand()
                new_hand.id = detected_hand.id
                new_hand.score = detected_hand.truth_value
                new_hand.fingertips = detected_hand.fingertips
                new_hand.intertips = detected_hand.intertips
                new_hand.positions = detected_hand.position_history
                new_hand.contour = detected_hand.contour
                new_hand.centerMass = detected_hand.center_of_mass
                new_hand.truthValue = detected_hand.truth_value
                new_hand.detected = detected_hand.detected
                new_hand.tracked = detected_hand.tracked
                ret.append(new_hand)
        print ret
        return ret

