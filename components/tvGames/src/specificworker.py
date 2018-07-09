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
        self.capture = cv2.VideoCapture(0)

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

        # frame = self.myqueue.get()
        ret, frame = self.capture.read()
        if ret:
            # ret, frame = self.readImg(self.stream)
            cv2.imshow('tvGame', frame)



            try:
                img = TImage(frame.shape[1], frame.shape[0], 3, ())
                img.image = frame.data
                people = self.handdetection_proxy.processImage(img)


            except Ice.Exception, e:
                traceback.print_exc()
                print e
        else:
            print "No video source"

        return True

