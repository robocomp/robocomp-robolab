#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
# Copyright (C) 2019 by YOUR NAME HERE
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
import pyrealsense2 as rs
import torch
import numpy as np
import cv2
from openpifpaf.network import nets
from openpifpaf import decoder, show, transforms
import argparse
import time
import PIL
from PySide2.QtCore import QMutexLocker

COCO_IDS = ["nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder", "right_shoulder", "left_elbow",
            "right_elbow", "left_wrist", "right_wrist", "left_hip", "right_hip", "left_knee", "right_knee",
            "left_ankle", "right_ankle"]
SKELETON_CONNECTIONS = [("left_ankle", "left_knee"),
                        ("left_knee", "left_hip"),
                        ("right_ankle", "right_knee"),
                        ("right_knee", "right_hip"),
                        ("left_hip", "right_hip"),
                        ("left_shoulder", "left_hip"),
                        ("right_shoulder", "right_hip"),
                        ("left_shoulder", "right_shoulder"),
                        ("left_shoulder", "left_elbow"),
                        ("right_shoulder", "right_elbow"),
                        ("left_elbow", "left_wrist"),
                        ("right_elbow", "right_wrist"),
                        ("left_eye", "right_eye"),
                        ("nose", "left_eye"),
                        ("nose", "right_eye"),
                        ("left_eye", "left_ear"),
                        ("right_eye", "right_ear"),
                        ("left_ear", "left_shoulder"),
                        ("right_ear", "right_shoulder")]


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.params = {}
        self.width = 0
        self.height = 0
        self.cameraid = 0
        self.adepth = []
        self.bdepth = []
        self.acolor = []
        self.bcolor = []
        self.points = []
        self.openpifpaf = False
        self.viewimage = False
        self.peoplelist = []
        self.timer.timeout.connect(self.compute)
        self.Period = 1

    def __del__(self):
        print('SpecificWorker destructor')
        if self.pipeline:
            self.pipeline.stop()

    def setParams(self, params):
        self.params = params
        self.width = int(self.params["width"])
        self.height = int(self.params["height"])
        self.cameraid = int(self.params["cameraid"])
        self.openpifpaf = "true" in self.params["openpifpaf"]
        self.verticalflip = "true" in self.params["verticalflip"]
        self.horizontalflip = "true" in self.params["horizontalflip"]
        self.viewimage = "true" in self.params["viewimage"]
        self.publishimage = "true" in self.params["publishimage"]
        self.depth_focal_x = int(self.params["depth_focal_x"])
        self.depth_focal_y = int(self.params["depth_focal_y"])
        self.color_focal_x = int(self.params["color_focal_x"])
        self.color_focal_y = int(self.params["color_focal_y"] )                                
        self.contFPS = 0
        self.start =  time.time()
        self.capturetime = time.time()
        self.initialize()
        self.timer.start(self.Period)
        return True

    def initialize(self):
        print("Initialize")
        # openpifpaf configuration
        class Args:
            source = 0
            checkpoint = None
            basenet = None
            dilation = None
            dilation_end = None
            headnets = ['pif', 'paf']
            dropout = 0.0
            quad = 1
            pretrained = False
            keypoint_threshold = None
            seed_threshold = 0.2
            force_complete_pose = False
            debug_pif_indices = []
            debug_paf_indices = []
            connection_method = 'max'
            fixed_b = None
            pif_fixed_scale = None
            profile_decoder = None
            instance_threshold = 0.05
            device = torch.device(type="cpu")
            disable_cuda = True
            scale = 1
            key_point_threshold = 0.05
            head_dropout = 0.0
            head_quad = 0
            default_kernel_size = 1
            default_padding = 0
            default_dilation = 1
            head_kernel_size = 1
            head_padding = 0
            head_dilation = 0
            cross_talk = 0.0
            two_scale = False
            multi_scale = False
            multi_scale_hflip = False
            paf_th = 0.1
            pif_th = 0.1
            decoder_workers = None
            experimental_decoder = False
            extra_coupling = 0.0

        self.args = Args()
        model, _ = nets.factory_from_args(self.args)
        model = model.to(self.args.device)
        # model.cuda()
        self.processor = decoder.factory_from_args(self.args, model)

        # realsense configuration
        try:
            config = rs.config()
            config.enable_device(self.params["device_serial"])
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, 30)
           
            self.pointcloud = rs.pointcloud()
            self.pipeline = rs.pipeline()
            cfg = self.pipeline.start(config)
#            profile = cfg.get_stream(rs.stream.color) # Fetch stream profile for depth stream
#            intr = profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
#            print (intr.fx, intr.fy)
#            depth_scale = cfg.get_device().first_depth_sensor().get_depth_scale()
#            print("Depth Scale is: " , depth_scale)
#            sys.exit(-1)
        except Exception as e:
            print("Error initializing camera")
            print(e)
            sysexit(-1)

    @QtCore.Slot()
    def compute(self):
        frames = self.pipeline.wait_for_frames()
        if not frames:
            return

        self.capturetime = time.time()
        depthData = frames.get_depth_frame()
        self.bdepth = np.asanyarray(depthData.get_data(), dtype=np.float32)
        self.bcolor = np.asanyarray(frames.get_color_frame().get_data())

        if self.horizontalflip:
            self.bcolor = cv2.flip(self.bcolor, 1)
            self.bdepth = cv2.flip(self.bdepth, 1)
        if self.verticalflip:
            self.bcolor = cv2.flip(self.bcolor, 0)
            self.bdepth = cv2.flip(self.bdepth, 0)

        #SWAP
        self.mutex.lock()
        self.acolor , self.bcolor = self.bcolor, self.acolor
        self.adepth , self.bdepth = self.bdepth, self.adepth
        self.mutex.unlock()



        if self.openpifpaf:
            self.processImage(0.3)
            self.publishData()

        
        if self.viewimage:
            cv2.imshow("Color_frame", self.color)
            cv2.setMouseCallback("Color_frame", self.mousecallback)
            cv2.waitKey(1)
            
        if self.publishimage:
            im = TImage()
            im.cameraID = self.cameraid
            im.width = self.width
            im.height = self.height
            im.focalx = self.color_focal_x 
            im.focaly = self.color_focal_y
            im.depth = 3
            im.image = self.acolor
            
            dep = TDepth()
            dep.cameraID = self.cameraid
            dep.width = self.width
            dep.height = self.height
            dep.focalx = self.depth_focal_x 
            dep.focaly = self.depth_focal_y  
            dep.depth = self.adepth
            
        
            try:
                dep.alivetime = (time.time() - self.capturetime)*1000
                im.alivetime = (time.time() - self.capturetime)*1000
                self.camerargbdsimplepub_proxy.pushRGBD(im, dep)
            except Exception as e:
                print("Error on camerabody data publication")
                print(e)

        if time.time() - self.start > 1:
                print("FPS:", self.contFPS)
                self.start = time.time()
                self.contFPS = 0
        self.contFPS += 1
        return True

    def processImage(self, scale):
        image = cv2.resize(self.acolor, None, fx=scale, fy=scale)
        image_pil = PIL.Image.fromarray(image)
        processed_image_cpu, _, __ = transforms.EVAL_TRANSFORM(image_pil, [], None)
        processed_image = processed_image_cpu.contiguous().to(non_blocking=True)
        fields = self.processor.fields(torch.unsqueeze(processed_image, 0))[0]

        keypoint_sets, _ = self.processor.keypoint_sets(fields)
        self.peoplelist = []
        # create joint dictionary
        for id, p in enumerate(keypoint_sets):
            person = Person()
            person.id = id
            person.joints = dict()
            for pos, joint in enumerate(p):
                if float(joint[2]) > 0.5:
                    keypoint = KeyPoint()
                    keypoint.i = int(joint[0] / scale)
                    keypoint.j = int(joint[1] / scale)
                    keypoint.score = float(joint[2])
                    
                    ki = keypoint.i - 320
                    kj = 240 - keypoint.j
                    pdepth = float(self.getDepth(keypoint.i, keypoint.j))
                    keypoint.z = pdepth   ## camara returns Z directly. If depth use equation
                    keypoint.x = ki*keypoint.z/self.focal
                    keypoint.y = kj*keypoint.z/self.focal
                    person.joints[COCO_IDS[pos]] = keypoint
            self.peoplelist.append(person)

            # draw
            if self.viewimage:
                for name1, name2 in SKELETON_CONNECTIONS:
                    try:
                        joint1 = person.joints[name1]
                        joint2 = person.joints[name2]
                        if joint1.score > 0.5:
                                cv2.circle(self.acolor, (joint1.i, joint1.j), 10, (0, 0, 255))
                        if joint2.score > 0.5:
                                cv2.circle(self.acolor, (joint2.i, joint2.j), 10, (0, 0, 255))
                        if joint1.score > 0.5 and joint2.score > 0.5:
                                cv2.line(self.acolor, (joint1.i, joint1.j), (joint2.i, joint2.j), (0, 255, 0), 2)
                    except:
                        pass

    #return median depth value
    def getDepth(self, i,j):
        OFFSET = 3
        values = []
        for xi in range(i-OFFSET,i+OFFSET):
            for xj in range(j-OFFSET, j+OFFSET):
                values.append(self.depth[xj, xi])
        return np.median(values)  #to mm REAL

######################################
##### PUBLISHER
######################################

    def publishData(self):
        people = PeopleData()
        people.cameraId = self.cameraid
        people.timestamp = time.time()
        people.peoplelist = self.peoplelist
#      print(people)
        try:
            self.humancamerabody_proxy.newPeopleData(people)
        except:
            print("Error on camerabody data publication")


    # =============== Methods for Component Implements ==================
    # ===================================================================
    ### CameraRGBDSimple
    #
    # getAll
    #
    def CameraRGBDSimple_getAll(self):
        #locker = QMutexLocker(self.mutex)
        im = TImage()
        im.cameraID = self.cameraid
        im.width = self.width
        im.height = self.height
        im.depth = 3
        im.focalx = self.color_focal_x 
        im.focaly = self.color_focal_y
        im.image = self.acolor
        
        dep = TDepth()
        dep.cameraID = self.cameraid
        dep.width = self.width
        dep.height = self.height
        dep.depth = self.adepth
        dep.focalx = self.depth_focal_x 
        dep.focaly = self.depth_focal_y
        im.alivetime = (time.time() - self.capturetime)*1000
        dep.alivetime = (time.time() - self.capturetime)*1000
        return im, dep

    #
    # getDepth
    #
    def CameraRGBDSimple_getDepth(self):
        #locker = QMutexLocker(self.mutex)
        dep = TDepth()
        dep.cameraID = self.cameraid
        dep.width = self.width
        dep.height = self.height
        dep.depth = self.adepth
        dep.focalx = self.depth_focal_x 
        dep.focaly = self.depth_focal_y
        dep.alivetime = (time.time() - self.capturetime)*1000
        return dep

    #
    # getImage
    #
    def CameraRGBDSimple_getImage(self):
        #locker = QMutexLocker(self.mutex)
        im = TImage()
        im.cameraID = self.cameraid
        im.width = self.width
        im.height = self.height
        im.focalx = self.color_focal_x 
        im.focaly = self.color_focal_y
        im.depth = 3
        im.image = self.acolor
        im.alivetime = (time.time() - self.capturetime)*1000
        return im

    ### RGBD ###
    #
    # getData
    #
    def RGBD_getData(self):
        #locker = QMutexLocker(self.mutex)
        hState = {}
        bState = TBaseState()

        return (self.acolor, self.adepth, hState, bState)

    #
    # getDepth
    #
    def RGBD_getDepth(self):
        #locker = QMutexLocker(self.mutex)
        dep.depth = self.adepth
        hState = {}
        bState = TBaseState()
        return (dep, hState, bState)

    #
    # getDepthInIR
    #
    def RGBD_getDepthInIR(self):
        #locker = QMutexLocker(self.mutex)
        print('not implemented')
        sys.exit(0)
        distanceMatrix = []
        hState = {}
        bState = TBaseState()
        return (distanceMatrix, hState, bState)

    #
    # getImage
    #
    def RGBD_getImage(self):
        #locker = QMutexLocker(self.mutex)
        print('not implemented')
        sys.exit(0)
        color = []
        depth = []
        points = []
        hState = {}
        bState = TBaseState()
        return (color, depth, points, hState, bState)

    #
    # getRGB
    #
    def RGBD_getRGB(self):
        #locker = QMutexLocker(self.mutex)
        hState = {}
        bState = TBaseState()
        return (self.color, hState, bState)

    #
    # getRGBDParams
    #
    def RGBD_getRGBDParams(self):
        #locker = QMutexLocker(self.mutex)
        print('not implemented')
        sys.exit(0)
        ret = TRGBDParams()
        return ret

    #
    # getRegistration
    #
    def RGBD_getRegistration(self):
        #locker = QMutexLocker(self.mutex)
        print('not implemented')
        sys.exit(0)
        ret = Registration()
        return ret

    #
    # getXYZ
    #
    def RGBD_getXYZ(self):
        locker = QMutexLocker(self.mutex)
        print('not implemented')
        sys.exit(0)
        points = []
        hState = {}
        bState = RoboCompGenericBase.TBaseState()
        return (points, hState, bState)

    #
    # getXYZByteStream
    #
    def RGBD_getXYZByteStream(self):
        #locker = QMutexLocker(self.mutex)
        print('not implemented')
        sys.exit(0)
        pointStream = []
        hState = {}
        bState = TBaseState()
        return (pointStream, hState, bState)

    #
    # setRegistration
    #
    def RGBD_setRegistration(self, value):
        #locker = QMutexLocker(self.mutex)
        print('not implemented')
        sys.exit(0)
        pass


    def mousecallback(self, event, x, y, flags, param):
        # grab references to the global variables
        global refPt, cropping
        Z = self.depth[y,x]
        X = (x-320)*Z/617
        Y = (y-240)*Z/616
        print("image",x, y,"point",X, Y, Z)
        # if the left mouse button was clicked, record the starting
        # (x, y) coordinates and indicate that cropping is being
        # performed
        
