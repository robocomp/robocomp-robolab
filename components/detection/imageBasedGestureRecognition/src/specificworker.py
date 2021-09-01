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
from RoboCompImageBasedGestureRecognition import *
import numpy as np
sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


from inference.ONNXAndTensorInference import ImageBasedRecognitionONNXInference, ImageBasedRecognitionONNXInference

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        self.weight = "src/_model/i3d_100_logit.onnx"

        # select inference model: pure pytorch, onnx, tensorrt
        self.estimator = ImageBasedRecognitionONNXInference(self.weight)

    def setParams(self, params):
        return True

    def compute(self):
        print('SpecificWorker.compute...')
        return True

    def startup_check(self):
        import time
        time.sleep(0.2)
        exit()



    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getGesture method from ImageBasedGestureRecognition interface
    #
    def ImageBasedGestureRecognition_getGesture(self, init_data):
        ret = GestureResult()
        arr = np.fromstring(init_data.images, np.uint8)
        videos = np.reshape(arr, (init_data.numFrames, init_data.height, init_data.width, init_data.depth))
        # TODO: preprocess TVIDEO input data
        gestureProb, gestureIndex = self.estimator(videos)
        ret.gestureProb = gestureProb
        ret.gestureIndex = gestureIndex

        return ret
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompImageBasedGestureRecognition you can use this types:
    # RoboCompImageBasedGestureRecognition.TVideo
    # RoboCompImageBasedGestureRecognition.GestureResult


