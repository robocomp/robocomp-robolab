#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2022 by YOUR NAME HERE
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

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import numpy as np
from numpy.random import choice as CColor # ChooseColor
import os
import time
import cv2
import queue
from threading import Thread
import json
import trt_pose.coco
import trt_pose.models
import copy
import torch
import torch2trt
from torch2trt import TRTModule
import torchvision.transforms as transforms
import PIL.Image
from trt_pose.parse_objects import ParseObjects

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 20
        if startup_check:
            self.startup_check()
        else:
            ########## user code ##################
            # config vars
            self.display = False
            self.human_parts_file = "human_pose.json"
            self.camera_name = "camera_top"
            self.model = 'densenet121_baseline_att_256x256_B_epoch_160.pth'
            self.optimized_model = 'densenet121_baseline_att_256x256_B_epoch_160_trt.pth'

            # Hz
            self.cont = 0
            self.last_time = time.time()
            self.fps = 0

            # trt
            self.TRT_MODEL_WIDTH = 256.
            self.TRT_MODEL_HEIGHT = 256.
            self.model_trt = TRTModule()
            self.model_trt.load_state_dict(torch.load(self.optimized_model))
            self.mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
            self.std = torch.Tensor([0.229, 0.224, 0.225]).cuda()


            # camera read thread
            self.read_queue = queue.Queue(1)
            self.read_thread = Thread(target=self.get_rgb_thread, args=["camera_top"], name="read_queue")
            self.read_thread.start()

            # result data
            self.write_queue = queue.Queue(1)
            #######################################
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        self.display = params["display"] == "true" or params["display"] == "True"

        self.human_parts_file = params["human_parts_file"]
        with open(self.human_parts_file, 'r') as f:
            self.human_pose = json.load(f)
        topology = trt_pose.coco.coco_category_to_topology(self.human_pose)
        self.parse_objects = ParseObjects(topology)

        self.camera_name = params["camera_name"]
        print("Params read. Starting...")
        return True


    @QtCore.Slot()
    def compute(self):
        rgb = self.read_queue.get()
        people_data = self.trtpose(rgb)
        try:
            self.write_queue.put_nowait(people_data)
        except:
            pass

        cv2.imshow("trt_pose", rgb)
        cv2.waitKey(1)

        # FPS
        self.show_fps()

        return True

####################################################################################################
    def get_rgb_thread(self, camera_name: str):
        while True:
            try:
                rgb = self.camerargbdsimple_proxy.getImage(camera_name)
                frame = np.frombuffer(rgb.image, dtype=np.uint8)
                frame = frame.reshape((rgb.height, rgb.width, 3))
                self.read_queue.put(frame)
            except:
                print("Error communicating with CameraRGBDSimple")
                return

    def trtpose(self, rgb):  # should be RGB
        img = cv2.resize(rgb, dsize=(int(self.TRT_MODEL_WIDTH), int(self.TRT_MODEL_HEIGHT)), interpolation=cv2.INTER_AREA)
        img = PIL.Image.fromarray(img)
        img = transforms.functional.to_tensor(img).to(torch.device('cuda'))
        img.sub_(self.mean[:, None, None]).div_(self.std[:, None, None])
        data = img[None, ...]
        #data = self.preprocess(img)
        cmap, paf = self.model_trt(data)
        cmap, paf = cmap.detach().cpu(), paf.detach().cpu()
        counts, objects, peaks = self.parse_objects(cmap, paf)  # , cmap_threshold=0.15, link_threshold=0.15)
        rgb_rows, rgb_cols, _ = rgb.shape
        for i in range(counts[0]):
            keypoints = self.get_keypoint(objects, i, peaks)
            for j in range(len(keypoints)):
                if keypoints[j][1]:
                    x = keypoints[j][2] * 640
                    y = keypoints[j][1] * 640
                    cv2.circle(rgb, (int(x), int(y)), 1, [0, 0, 255], 2)

    # def preprocess(self, image):
    #     global _device
    #     image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    #     image = PIL.Image.fromarray(image)
    #     image = transforms.functional.to_tensor(image).to(_device)
    #     image.sub_(self.mean[:, None, None]).div_(self.std[:, None, None])
    #     return image[None, ...]

    def get_keypoint(self, humans, hnum, peaks):
        kpoint = []
        human = humans[0][hnum]
        C = human.shape[0]
        for j in range(C):
            k = int(human[j])
            if k >= 0:
                peak = peaks[0][j][k]   # peak[1]:width, peak[0]:height
                peak = (j, float(peak[0]), float(peak[1]))
                kpoint.append(peak)
            else:
                peak = (j, None, None)
                kpoint.append(peak)
        return kpoint
####################################################################################################
    def show_fps(self):
        if time.time() - self.last_time > 1:
            self.last_time = time.time()
            print("Freq: ", self.cont, "Hz. Waiting for image")
            self.cont = 0
        else:
            self.cont += 1

    def startup_check(self):
        print(f"Testing RoboCompHumanCameraBodyPub.TImage from ifaces.RoboCompHumanCameraBodyPub")
        test = ifaces.RoboCompHumanCameraBodyPub.TImage()
        print(f"Testing RoboCompHumanCameraBodyPub.TGroundTruth from ifaces.RoboCompHumanCameraBodyPub")
        test = ifaces.RoboCompHumanCameraBodyPub.TGroundTruth()
        print(f"Testing RoboCompHumanCameraBodyPub.KeyPoint from ifaces.RoboCompHumanCameraBodyPub")
        test = ifaces.RoboCompHumanCameraBodyPub.KeyPoint()
        print(f"Testing RoboCompHumanCameraBodyPub.Person from ifaces.RoboCompHumanCameraBodyPub")
        test = ifaces.RoboCompHumanCameraBodyPub.Person()
        print(f"Testing RoboCompHumanCameraBodyPub.PeopleData from ifaces.RoboCompHumanCameraBodyPub")
        test = ifaces.RoboCompHumanCameraBodyPub.PeopleData()
        print(f"Testing RoboCompCameraRGBDSimple.TImage from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TImage()
        print(f"Testing RoboCompCameraRGBDSimple.TDepth from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TDepth()
        print(f"Testing RoboCompCameraRGBDSimple.TRGBD from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TRGBD()
        print(f"Testing RoboCompHumanCameraBody.TImage from ifaces.RoboCompHumanCameraBody")
        test = ifaces.RoboCompHumanCameraBody.TImage()
        print(f"Testing RoboCompHumanCameraBody.TGroundTruth from ifaces.RoboCompHumanCameraBody")
        test = ifaces.RoboCompHumanCameraBody.TGroundTruth()
        print(f"Testing RoboCompHumanCameraBody.KeyPoint from ifaces.RoboCompHumanCameraBody")
        test = ifaces.RoboCompHumanCameraBody.KeyPoint()
        print(f"Testing RoboCompHumanCameraBody.Person from ifaces.RoboCompHumanCameraBody")
        test = ifaces.RoboCompHumanCameraBody.Person()
        print(f"Testing RoboCompHumanCameraBody.PeopleData from ifaces.RoboCompHumanCameraBody")
        test = ifaces.RoboCompHumanCameraBody.PeopleData()
        QTimer.singleShot(200, QApplication.instance().quit)

    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getAll method from CameraRGBDSimple interface
    #
    def CameraRGBDSimple_getAll(self, camera):
        ret = ifaces.RoboCompCameraRGBDSimple.TRGBD()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of getDepth method from CameraRGBDSimple interface
    #
    def CameraRGBDSimple_getDepth(self, camera):
        ret = ifaces.RoboCompCameraRGBDSimple.TDepth()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of getImage method from CameraRGBDSimple interface
    #
    def CameraRGBDSimple_getImage(self, camera):
        ret = ifaces.RoboCompCameraRGBDSimple.TImage()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of newPeopleData method from HumanCameraBody interface
    #
    def HumanCameraBody_newPeopleData(self):
        ret = ifaces.RoboCompHumanCameraBody.PeopleData()
        #
        # write your CODE here
        #
        return ret
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompCameraRGBDSimplePub you can publish calling this methods:
    # self.camerargbdsimplepub_proxy.pushRGBD(...)

    ######################
    # From the RoboCompHumanCameraBodyPub you can publish calling this methods:
    # self.humancamerabodypub_proxy.newPeopleData(...)

    ######################
    # From the RoboCompHumanCameraBodyPub you can use this types:
    # RoboCompHumanCameraBodyPub.TImage
    # RoboCompHumanCameraBodyPub.TGroundTruth
    # RoboCompHumanCameraBodyPub.KeyPoint
    # RoboCompHumanCameraBodyPub.Person
    # RoboCompHumanCameraBodyPub.PeopleData

    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # RoboCompCameraRGBDSimple.TImage
    # RoboCompCameraRGBDSimple.TDepth
    # RoboCompCameraRGBDSimple.TRGBD

    ######################
    # From the RoboCompHumanCameraBody you can use this types:
    # RoboCompHumanCameraBody.TImage
    # RoboCompHumanCameraBody.TGroundTruth
    # RoboCompHumanCameraBody.KeyPoint
    # RoboCompHumanCameraBody.Person
    # RoboCompHumanCameraBody.PeopleData


