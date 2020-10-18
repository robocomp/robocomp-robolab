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

from genericworker import *
import numpy as np
import cv2

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

## Change to path of Python it is not defualt
try:
    sys.path.append('/usr/local/python')
    from openpose import pyopenpose as op
except ImportError as e:
    print("Error importing openpose. Check it's installation")
    raise e

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 20
        self.timer.start(self.Period)

        ## Change to openpose installation path (Default considered to be HOME)
        self.openpose_path = os.getenv("HOME") + '/openpose'
        params = dict()
        params["model_folder"] = self.openpose_path + "/models"
        params["hand"] = True
        params["hand_detector"] = 2
        params["body"] = 0

        try:
            self.openpose_wrapper = op.WrapperPython()
            self.openpose_wrapper.configure(params)
            self.openpose_wrapper.start()
        except:
            print("Error creating python wrapper of openpose. Check installation")
            sys.exit(-1)

        print("Component Started")

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):

        return True


    @QtCore.Slot()
    def compute(self):

        return True

    # Converts a hand bounding box into a OpenPose Rectangle
    def box2oprectangle(self,box):
        [left, right, top, bottom] = box
        width = np.abs(right - left)
        height = np.abs(top - bottom)
        max_length = int(max(width,height))
        center = (int(left + (width /2 )), int(bottom - (height/2)))
        # Openpose hand detector needs the bounding box to be quite big , so we make it bigger
        # Top point for rectangle
        new_top = (int(center[0] - max_length / 1.3), int(center[1] - max_length /1.3))
        max_length = int(max_length * 1.6)
        hand_rectangle = op.Rectangle(new_top[0],new_top[1],max_length,max_length)
        return hand_rectangle

    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # getKeypoints
    #
    def HandKeypoint_getKeypoints(self, HandImage, box):
        #
        # implementCODE
        #
        # keypoints = KeypointType()
        arr = np.fromstring(HandImage.image, np.uint8)
        frame = np.reshape(arr, (HandImage.height, HandImage.width, HandImage.depth))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        print('Input Data Recieved')
        box = np.array(box)
        ## Creating openpose rectangle
        hands_rectangles = [[self.box2oprectangle(box),op.Rectangle(0., 0., 0., 0.)]]

        # Create new datum
        datum = op.Datum()
        datum.cvInputData = frame
        datum.handRectangles = hands_rectangles
        # Process and display image
        self.openpose_wrapper.emplaceAndPop([datum])

        if datum.handKeypoints[0].shape == ():
            # if there were no detections
            hand_keypoints = None
        else:
            hand_keypoints = datum.handKeypoints[0][0]
        return hand_keypoints

    # ===================================================================
    # ===================================================================
