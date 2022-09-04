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
import sys

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import numpy as np
import os
import time
import cv2
import queue
import traceback
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

_CONNECTIONS = [[15, 13], [13, 11], [16, 14], [14, 12], [11, 12], [5, 7], [6, 8], [7, 9], [8, 10],
                [1, 2], [0, 1], [0, 2], [1, 3], [2, 4], [3, 5], [4, 6], [17, 0], [17, 5], [17, 6],
                [17, 11], [17, 12]]

_JOINT_NAMES = ["nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder",
                "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist",
                "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle", "neck"]

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 33
        if startup_check:
            self.startup_check()
        else:
            ########## user code ##################
            # config vars
            self.display = False
            self.with_objects = True
            self.human_parts_file = "human_pose.json"
            self.camera_name = "camera_top"
            self.model = 'densenet121_baseline_att_256x256_B_epoch_160.pth'
            self.optimized_model = 'densenet121_baseline_att_256x256_B_epoch_160_trt.pth'

            # camera
            try:
                rgb = self.camerargbdsimple_proxy.getImage(self.camera_name)
                self.image_focalx = rgb.focalx
                self.image_focaly = rgb.focaly
                print("CameraRGBDSimple contacted: rows:", rgb.height, " cols:", rgb.width)
            except:
                print("No connection to CameraRGBDSimple. Aborting")
                sys.exit()
            self.image_captured_time = 0

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
            self.read_image_queue = queue.Queue(1)
            self.write_image_queue = queue.Queue(1)
            self.read_thread = Thread(target=self.get_rgb_thread, args=[self.camera_name], name="read_camera_queue")
            self.read_thread.start()
            self.people_data_write = ifaces.RoboCompHumanCameraBody.PeopleData()
            self.people_data_read = ifaces.RoboCompHumanCameraBody.PeopleData()

            # objects read thread
            self.read_objects_queue = queue.Queue(1)
            self.write_objects_queue = queue.Queue(1)
            self.read_thread = Thread(target=self.get_objects_thread, name="read_objects_queue", daemon=True)
            self.read_thread.start()

            # result data
            self.write_pose_queue = queue.Queue(1)
            #######################################
            
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        # TODO: move to init
        self.display = params["display"] == "true" or params["display"] == "True"
        self.with_objects = params["with_objects"] == "true" or params["with_objects"] == "True"
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
        rgb = self.read_image_queue.get()
        person_rois = []
        if self.with_objects:
            data = self.read_objects_queue.get()
            try:
                data = self.yoloobjects_proxy.getYoloObjects()
            except Ice.Exception as e:
                traceback.print_exc()
                return False

            # get person's rois from data.objects and copy on black image
            black = np.zeros(rgb.shape, dtype=np.uint8)
            for object in data.objects:
                if object.type == 0:  #person
                    black[object.top:object.bot, object.left: object.right, :] = \
                        rgb[object.top:object.bot, object.left: object.right, :]
                    person_rois.append([object.left, object.top, object.right-object.left, object.bot-object.top, object.id])
            self.people_data_write = self.trtpose(black, person_rois)

        else:
            self.people_data_write = self.trtpose(rgb, person_rois)

        self.people_data_write, self.people_data_read = self.people_data_read, self.people_data_write

        #cv2.imshow("trt_pose", rgb)
        #cv2.waitKey(1)

        # FPS
        self.show_fps()
        
        return True

####################################################################################################
    def get_rgb_thread(self, camera_name: str):
        while True:
            try:
                rgb = self.camerargbdsimple_proxy.getImage(camera_name)
                self.image_captured_time = time.time()
                frame = np.frombuffer(rgb.image, dtype=np.uint8)
                frame = frame.reshape((rgb.height, rgb.width, 3))
                self.read_image_queue.put(frame)
            except:
                print("Error communicating with CameraRGBDSimple")
                traceback.print_exc()
                break

    def get_objects_thread(self):
        while True:
            try:
                data = self.yoloobjects_proxy.getYoloObjects()
                self.read_objects_queue.put(data)
            except:
                print("Error communicating with YoloObjects")
                traceback.print_exc()
                break

    def trtpose(self, rgb, rois):  # should be RGB 416x416
        img = cv2.resize(rgb, dsize=(int(self.TRT_MODEL_WIDTH), int(self.TRT_MODEL_HEIGHT)),
                         interpolation=cv2.INTER_AREA)
        img = PIL.Image.fromarray(img)
        img = transforms.functional.to_tensor(img).to(torch.device('cuda'))
        img.sub_(self.mean[:, None, None]).div_(self.std[:, None, None])
        data = img[None, ...]
        cmap, paf = self.model_trt(data)
        cmap, paf = cmap.detach().cpu(), paf.detach().cpu()
        counts, objects, peaks = self.parse_objects(cmap, paf)  # , cmap_threshold=0.15, link_threshold=0.15)
        rgb_rows, rgb_cols, _ = rgb.shape
        people_data = ifaces.RoboCompHumanCameraBody.PeopleData(timestamp=time.time())
        people_data.peoplelist = []
        votes = np.zeros((counts[0], len(rois)), dtype=float)   # vote accumulator
        for i in range(counts[0]):
            keypoints = self.get_keypoint(objects, i, peaks)
            joints = dict()
            for id in range(len(keypoints)):
                if keypoints[id][1] and keypoints[id][2]:
                    x = keypoints[id][2] * 640
                    y = keypoints[id][1] * 640
                    #cv2.circle(rgb, (int(x), int(y)), 1, [0, 0, 255], 2)
                    joints[_JOINT_NAMES[id]] = ifaces.RoboCompHumanCameraBody.KeyPoint(i=x, j=y)
                    if self.with_objects:
                        self.vote_for_roi(rois, (x, y), votes[i])
            people_data.peoplelist.append(ifaces.RoboCompHumanCameraBody.Person(id=i, joints=joints))

        # select the most voted ROI
        if self.with_objects:
            roi_winners = np.argmax(votes, axis=1)
            for k, person in enumerate(people_data.peoplelist):
                person.id = rois[roi_winners[k]][4]

        return people_data

    def vote_for_roi(self, rois, pt, person_votes):
        for k, rect in enumerate(rois):
            if rect[0] < pt[0] <= rect[0] + rect[2] and rect[1] <= pt[1] < rect[1] + rect[3]:
                person_votes[k] += 1

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
        #img = self.write_image_queue.get()
        #ret = ifaces.RoboCompCameraRGBDSimple.TImage(
            #compressed=False,
            #cameraID=0,
            #height=img.shape[0],
            #width=img.shape[1],
            #depth=img.shape[2],
            #focalx=self.image_focalx,
            #focaly=self.image_focaly,
            #alivetime=self.image_captured_time - time.time(),
            #image=img.tobytes()
        #)
        return ret
    #
    # IMPLEMENTATION of newPeopleData method from HumanCameraBody interface
    #
    def HumanCameraBody_newPeopleData(self):
        return self.people_data_read

    # IMPLEMENTATION of getJointData method from HumanCameraBody interface
    #######################################################################

    def HumanCameraBody_getJointData(self):
        print("Returning joint data")
        ret = ifaces.RoboCompHumanCameraBody.TJointData()
        ret.jointNames = {}
        for i, jnt in enumerate(_JOINT_NAMES):
            ret.jointNames[i] = jnt
        ret.connections = []
        for a, b in _CONNECTIONS:
            conn = ifaces.RoboCompHumanCameraBody.TConnection(first=a, second=b)
            ret.connections.append(conn)

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


