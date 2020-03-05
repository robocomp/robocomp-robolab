#
# Copyright (C) 2017 by YOUR NAME HERE
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

import sys
import os
import traceback
import time
import numpy as np
import cv2
from PySide import QtCore
from genericworker import *


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 50
        self.source = 0
        self.timer.start(self.Period)

    def setParams(self, params):
        self.source = params["source"]
        if self.source.isdigit():
            self.source = int(self.source)
        self.Period = int(params["period"])
        self.display = "true" in params["display"]

        self.capL = cv2.VideoCapture(self.source)
        return True

    @QtCore.Slot()
    def compute(self):
        retL, self.frameL = self.capL.read()
        if retL:
            rows, cols, depth = self.frameL.shape
        else:
            print("No frame could be read")

        # Display the resulting frame
        if self.display:
            cv2.imshow('frameL', self.frameL)
        return True

    #
    # SERVANTS ---------------------  getImage
    #
    def getImage(self):
        #
        # implementCODE
        #
        im = TImage()
        im.image = self.frameL
        im.height, im.width, im.depth = self.frameL.shape
        return im
