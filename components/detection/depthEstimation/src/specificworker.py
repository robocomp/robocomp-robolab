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

import numpy as np
import cv2
import sys
import os
import time
import copy
import glob
import argparse
import matplotlib
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '5'

from keras.models import load_model
from layers import BilinearUpSampling2D
from utils import predict, load_images, display_images
from genericworker import *

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 20
        self.timer.start(self.Period)
        self.method = 'TL'
       
        if(self.method=='TL'):
            try:
                  model_path = 'assets/nyu.h5' 
                  # Custom object needed for inference and training
                  custom_objects = {'BilinearUpSampling2D': BilinearUpSampling2D, 'depth_loss_function': None}

                  print('Loading model...')
 
                  # Load model into GPU / CPU
                  self.model = load_model(model_path, custom_objects=custom_objects, compile=False)

                  print('\nModel loaded ({0}).'.format(model_path))
        
            except:
                  print("Error Loading Model. Ensure that models are downloaded and placed in correct directory")
                    

        print("Component Started")

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):

        return True


    @QtCore.Slot()
    def compute(self):
 
        return True



    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # getDepthEstimation
    #
    def DepthEstimation_getDepthEstimation(self, depthImg):
        #
        # implementCODE
        #
        try:
            # Rearranging to form numpy matrix
            frame = np.fromstring(depthImg.image, np.uint8)
            # Resizing to required size
            # depthImg_shape = (480,640,3)
            frame = np.reshape(frame, (depthImg.height, depthImg.width, depthImg.depth))
            #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            outputs = None
            
            if(self.method=='TL'):
                inputs = load_images(frame)
                #print('\n  Frame Loaded images of size {1}.'.format(inputs.shape[1:]))
                outputs = predict(self.model, inputs)
                
                if(outputs is not None):
                    print('depth predicted')
                else:
                    print("No depth predicted")
            else:
                print("Error! Please enter valid depth estimation method ")             

        except Exception as e:
            print(e)
            print("Error processing input image")
        # Create new DepthScene for depth output
        # outputs_shape = (1,240,480,1) 
        result = DepthScene()
        result.value = outputs
        result.height,result.width,result.depth = outputs[0].shape
 
        return result 
        

    # ===================================================================
    # ===================================================================
