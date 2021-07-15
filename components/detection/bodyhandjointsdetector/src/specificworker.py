#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
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

from rich.console import Console
from genericworker import *
from RoboCompBodyHandJointsDetector import *
from .inference.ONNXAndTensorInference import BodyDetectorONNXInference, BodyDetectorONNXTensorRTInference
import numpy as np

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 2000
        self.timer.start(self.Period)

        #TODO: get weight from proxy here
        self.weight = ""

        # init ONNX inference model.
        self.estimator = BodyDetectorONNXInference(self.weight)

        # init TensorRT inference model.
        # self.estimator = BodyDetectorONNXTensorRTInference(self.weight)

    def __del__(self):
        console.print('SpecificWorker destructor')

    def setParams(self, params):
        return True

    @QtCore.Slot()
    def compute(self):
        return True

    def startup_check(self):
        import time
        time.sleep(0.2)
        exit()

    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getBodyAndHand method from BodyHandJointsDetector interface
    #
    def BodyHandJointsDetector_getBodyAndHand(self, init_data):
        # parsing image.
        arr = np.fromstring(init_data.image, np.uint8)
        frame = np.reshape(arr, (init_data.height, init_data.width, init_data.depth))

        body = self.estimator(frame)

        return ListFullBody(body)
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompBodyHandJointsDetector you can use this types:
    # RoboCompBodyHandJointsDetector.TImage
    # RoboCompBodyHandJointsDetector.TBody


