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
from .inference.ONNXAndTensorInference import PoseDetectionONNXTensorRT, PoseDetectionONNX
from RoboCompPoseBasedGestureRecognition import *

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
        #
        # if torch.cuda.is_available():
        # 	self.device = 'cuda:0'
        # else:
        # 	self,device = 'cpu'
        self.weight = ""

        # select inference model: pure pytorch, onnx, tensorrt
        self.estimator = PoseDetectionONNXTensorRT(self.weight)


    def setParams(self, params):
        return True

    @QtCore.Slot()
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
    # IMPLEMENTATION of getBodyAndHand method from PoseBasedGestureRecognition interface
    #
    def PoseBasedGestureRecognition_getGesture(self, input):
        ret = GestureResult()
        #
        # write your CODE here
        #
        gestureProb, gestureIndex = self.estimator(input)
        ret.gestureProb = gestureProb
        ret.gestureIndex = gestureIndex

        return ret
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompPoseBasedGestureRecognition you can use this types:
    # RoboCompPoseBasedGestureRecognition.Pose
    # RoboCompPoseBasedGestureRecognition.GestureResult


