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


from genericworker import *
sys.path.append('/opt/robocomp/lib')
import gi
gi.require_version('Gtk', '2.0')    
import numpy as np
from numpy.random import choice as CColor # ChooseColor
import os
import time
import cv2
#from colored import fg
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

device = torch.device('cuda')

class FPSCounter():
    def __init__(self):
        self.start = time.time()
        self.counter = 0
    def count(self, period=1):
        self.counter += 1
        if time.time() - self.start > period:
            print("Skeleton Detector - Freq -> ", self.counter, "Hz")
            self.counter = 0
            self.start = time.time()

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        self.model = 'densenet'
        self.IMPUT_WIDTH  = 640
        self.INPUT_HEIGHT = 480
        self.draw = True
        self.rotate = True
        self.skeleton_points = []
        self.skeleton_white = []
        self.depth_scale = 0.0
        self.skeleton_img = None
        self.display = False
        self.people_data = None
       
        with open('human_pose.json', 'r') as f:
            self.human_pose = json.load(f)

        topology = trt_pose.coco.coco_category_to_topology(self.human_pose)
        num_parts = len(self.human_pose['keypoints'])
        num_links = len(self.human_pose['skeleton'])

        if self.model == 'resnet':
            MODEL_WEIGHTS = '../resnet18_baseline_att_224x224_A_epoch_249.pth'
            OPTIMIZED_MODEL = 'resnet18_baseline_att_224x224_A_epoch_249_trt.pth'
            model = trt_pose.models.resnet18_baseline_att(num_parts, 2 * num_links).cuda().eval()
            self.MODEL_WIDTH = 224.
            self.MODEL_HEIGHT = 224.
        elif self.model == 'densenet':
            MODEL_WEIGHTS = '../densenet121_baseline_att_256x256_B_epoch_160.pth'
            OPTIMIZED_MODEL = 'densenet121_baseline_att_256x256_B_epoch_160_trt.pth'
            model = trt_pose.models.densenet121_baseline_att(num_parts, 2 * num_links).cuda().eval()
            self.MODEL_WIDTH = 256.
            self.MODEL_HEIGHT = 256.

        # Preparamos un modelo optimizado si no existe. Si si existe, solo lo cargamos
        data = torch.zeros((1, 3, int(self.MODEL_HEIGHT), int(self.MODEL_WIDTH))).cuda()
        if os.path.exists(OPTIMIZED_MODEL) == False:
            #print("Preparando modelo optimizado. Espere...")
            model.load_state_dict(torch.load(MODEL_WEIGHTS))
            self.model_trt = torch2trt.torch2trt(model, [data], fp16_mode=True, max_workspace_size=1<<25)
            torch.save(self.model_trt.state_dict(), OPTIMIZED_MODEL)

        self.model_trt = TRTModule()
        self.model_trt.load_state_dict(torch.load(OPTIMIZED_MODEL))

        # Popurri extraño quer aun no se sabe que hace :p
        self.mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
        self.std = torch.Tensor([0.229, 0.224, 0.225]).cuda()

        self.parse_objects = ParseObjects(topology)

        # time
        self.fps = FPSCounter()

    def __del__(self):
        """Destructor"""
        
    def setParams(self, params):
        try:
            self.display = params["display"] == "true" or params["display"] == "True"
        except:
            print("Error reading config params")
        return True

    def compute(self):
        while True:
            rgbd = self.camerargbdsimple_proxy.getAll("camera_top")
            self.people_data = self.compute_body(rgbd.image, rgbd.depth)
            self.fps.count()
            if self.display:
                cv2.imshow("color", self.skeleton_img)
                cv2.waitKey(1)

    #######################################################################################
    def compute_body(self, color, depth):
        image = np.frombuffer(color.image, np.uint8).reshape(color.height, color.width, color.depth)
        depth_image = np.frombuffer(depth.depth, np.float32).reshape(depth.height, depth.width, 1)
        center_x = color.width / 2
        center_y = color.height / 2

        # copy to a squared image so the resize does not warps the image
        # square_img = np.zeros([color.height, color.height, 3], np.uint8)
        # x_offset = (color.height - color.width)//2
        # square_img[0:image.shape[0], x_offset:x_offset + image.shape[1]] = image
        img = cv2.resize(image, dsize=(int(self.MODEL_WIDTH), int(self.MODEL_HEIGHT)), interpolation=cv2.INTER_AREA)
        data = self.preprocess(img)
        cmap, paf = self.model_trt(data)
        cmap, paf = cmap.detach().cpu(), paf.detach().cpu()
        counts, objects, peaks = self.parse_objects(cmap, paf)  # , cmap_threshold=0.15, link_threshold=0.15)

        keypoints_names = self.human_pose["keypoints"]
        if self.display:
            self.skeleton_img = copy.deepcopy(image)
        people_data = RoboCompHumanCameraBody.PeopleData()
        people_data.timestamp = time.time()
        people_data.peoplelist = []

        for i in range(counts[0]):
            available_joints_counter = 0
            new_person = RoboCompHumanCameraBody.Person()
            new_person.id = i
            TJoints = {}
            bounding_list = []
            keypoints = self.get_keypoint(objects, i, peaks)
            for kpoint in range(len(keypoints)):
                key_point = RoboCompHumanCameraBody.KeyPoint()
                if keypoints[kpoint][1] != None and keypoints[kpoint][2] != None:
                    available_joints_counter += 1
                    key_point.i = int(keypoints[kpoint][2] * color.width)  # camera is vertical
                    key_point.j = int(keypoints[kpoint][1] * color.height)
                    key_point.y = float(depth_image[key_point.j, key_point.i])
                    key_point.z = -(key_point.y / depth.focalx) * (key_point.j - center_y)
                    key_point.x = (key_point.y / depth.focaly) * (key_point.i - center_x)
                    TJoints[str(kpoint)] = key_point
                    bounding_list.append([key_point.i, key_point.j])
                    if self.display:
                        cv2.circle(self.skeleton_img, (key_point.i, key_point.j), 1, [0, 255, 0], 2)

            if available_joints_counter < 3:
                continue
            # compute ROI
            if len(bounding_list) > 0:
                bx, by, bw, bh = cv2.boundingRect(np.array(bounding_list))
                new_person.roi = RoboCompHumanCameraBody.TImage()
                temp_img = cv2.resize(image[by:by + bh, bx:bx + bw], dsize=(int(150), int(300)),
                                      interpolation=cv2.INTER_AREA)
                new_person.roi.width = temp_img.shape[0]
                new_person.roi.height = temp_img.shape[1]
                new_person.roi.image = temp_img.tobytes()
            new_person.joints = TJoints
            people_data.peoplelist.append(new_person)
        return people_data

    def startup_check(self):
        pass

    #########################################################################################
    """                                     INTERFACE METHODS                            """
    #########################################################################################
    def HumanCameraBody_newPeopleData(self):
        """
        Returns a valid people data_structure
        """

        if self.people_data != None:
            return self.people_data

    #########################################################################################
    """                                     UTILITY METHODS                               """
    #########################################################################################
    def preprocess(self, image):
        global device
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = PIL.Image.fromarray(image)
        image = transforms.functional.to_tensor(image).to(device)
        image.sub_(self.mean[:, None, None]).div_(self.std[:, None, None])
        return image[None, ...]
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
   

