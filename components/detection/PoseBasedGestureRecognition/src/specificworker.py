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
from RoboCompPoseBasedGestureRecognition import *
import numpy as np
sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

from inference.ONNXAndTensorInference import PoseBasedRecognitionONNXInference
# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.weight = "src/_model/poseTGN_100_logit.onnx"

        # select inference model: pure pytorch, onnx, tensorrt
        self.estimator = PoseBasedRecognitionONNXInference(self.weight)

    def __del__(self):
        console.print('SpecificWorker destructor')

    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getBodyAndHand method from PoseBasedGestureRecognition interface
    #
    def PoseBasedGestureRecognition_getBodyAndHand(self, poses):
        ret = GestureResult()

        pose_list = []
        for ele in poses:
            parsed_pose = np.reshape(ele.keyPoints, (55,2))
            pose_list.append(parsed_pose)
        pose_list= np.concatenate(pose_list, axis = 1)
        pose_list = pose_list[None, :, :]

        gestureProb, gestureIndex = self.estimator(pose_list)
        ret.gestureProb = gestureProb
        ret.gestureIndex = gestureIndex

        return ret
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompPoseBasedGestureRecognition you can use this types:
    # RoboCompPoseBasedGestureRecognition.TBody
    # RoboCompPoseBasedGestureRecognition.GestureResult


