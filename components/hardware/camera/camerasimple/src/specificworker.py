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

import sys, os, traceback, time
import numpy as np
import cv2
from PySide2.QtCore import qApp, QTimer
from genericworker import *


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 50
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        self.capL = cv2.VideoCapture(0)
        return True


    @QtCore.Slot()
    def compute(self):
        # print 'SpecificWorker.compute...'

        retL, self.frameL = self.capL.read()
        if retL:
            rows,cols,depth =  self.frameL.shape
        else:
            print ("No frame could be read")


        # Display the resulting frame
        cv2.imshow('frameL',self.frameL)
        return True

    def startup_check(self):
        QTimer.singleShot(200, qApp.quit)



    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # getImage
    #
    def CameraSimple_getImage(self):
        #
        #implementCODE
        #
        im = TImage()
        im.image = self.frameL
        im.height, im.width, im.depth = self.frameL.shape
        return im

    # ===================================================================
    # ===================================================================


