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
import sys
import os
import cv2
import time
import copy
import glob
import argparse
import matplotlib
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '5'

from display import display_images
from matplotlib import pyplot as plt
from genericworker import *

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
##sys.path.append(os.path.join(os.getcwd(),"assets"))  

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 20
        self.timer.start(self.Period)
        self.method = 'TL'
        # Method = 'ADA' for depth estimation using Adabins
        # Method = 'TL' for depth estimation using Transfer Learning
      
        ##if(self.method=='TL'):
        ##    try:
        ##        # Argument Parser
                  ##parser = argparse.ArgumentParser(description='High Quality Monocular Depth ##Estimation via Transfer Learning')
                  ##parser.add_argument('--model', default='nyu.h5', type=str, help='Trained Keras model ##file.')
                  ##args = parser.parse_args()
        ##          model_path = 'nyu.h5' 
                  # Custom object needed for inference and training
        ##          custom_objects = {'BilinearUpSampling2D': BilinearUpSampling2D, ##'depth_loss_function': None}

        ##          print('Loading model...')
 
                  # Load model into GPU / CPU
        ##          self.model = load_model(model_path, custom_objects=custom_objects, compile=False)

        ##          print('\nModel loaded ({0}).'.format(model_path))
        
        ##    except:
        ##          print("Error Loading Model. Ensure that models are downloaded and placed in correct ##directory")

        # display configurations
        self.thickness = 2
        self.fps_text_color = (0,0,0)
        
        # storing program runtime and processed frames for calculating FPS
        self.start_time = 0
     
        print('Component Started')
        
    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        return True

    @QtCore.Slot()
    def compute(self):
        self.start_time = time.time()
        try:
            data = self.camerasimple_proxy.getImage()
        except:
            print("Error taking camera feed. Make sure Camerasimple is up and running")
            return False
        # Rearranging to form numpy matrix
        frame = np.fromstring(data.image, np.uint8)
        frame = np.reshape(frame, (data.height, data.width, data.depth))
        #frame_shape = (480,640,3)
        
        #Create new TImage
        depthinput = RoboCompDepthEstimation.TImage()
        depthinput.image = frame
        depthinput.height, depthinput.width, depthinput.depth = frame.shape
        
        #Obtaining depth output from input depthinput
        self.depthData = self.depthestimation_proxy.getDepthEstimation(depthinput)
        depthData = self.DepthEstimationClient_getDepthEstimation(data)
        
        #Displaying final depthmap
        self.display(data,self.depthData)
        
        return True
        
    def display(self,depthImg,depthData):
        elapsed_time = time.time() - self.start_time
        out = depthData.value
        
        #Converting into numpy array/matrix
        frame = np.asarray(out)
        frame = np.reshape(frame, (1,depthData.height, depthData.width, 1))
        #frame_shape = (1,240,320,1)
        
        ##frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        #frame = cv2.resize(frame,(640,480))
        
        #displaying depthmap frame         
        viz = display_images(frame.copy()) 
        print("displaying depth frame")
        #print(viz.shape)
        
        ## Display image using OpenCV
        viz = cv2.resize(viz,(640,480))
        
        ## insert FPS in frame
        ###fps = 1.0 / elapsed_time
        ###print(elapsed_time)
        ###print_text = "FPS : " + str(int(fps))
        ###cv2.putText(viz, print_text, (20, 50),
        ###        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.fps_text_color, 2)
        cv2.imshow("Depth Estimation Client",viz)
        
        
    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # getDepthEstimation
    #
    def DepthEstimationClient_getDepthEstimation(self, depthImg):
        #
        # implementCODE
        #
        try:
            # Rearranging to form numpy matrix
            frame = np.fromstring(depthImg.image, np.uint8)
            frame = np.reshape(frame, (depthImg.height, depthImg.width, depthImg.depth))
            #frame_cp = copy.deepcopy(frame)
            
            outputs = None
            if(self.method=='TL'):
                #inputs = load_images(frame)
                #print('\n  Frame Loaded images of size {1}.'.format(inputs.shape[1:]))
                outputs = self.depthData.value
                if(outputs is not None):
                    print('depth predicted')
                else:
                    print("No depth predicted")
            else:
                print("Error! Please enter valid detection method number")

        except Exception as e:
            print(e)
            print("Error processing input image")
        
        depth = RoboCompDepthEstimationClient.Depth()
        depth.output = outputs
        depth.height,depth.width,depth.depth = self.depthData.height,self.depthData.width,self.depthData.depth
        
        return depth
        
    # ===================================================================
    # =================================================================== 
