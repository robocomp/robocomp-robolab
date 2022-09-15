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
from threading import Thread, Event
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

# face
sys.path.append('/home/robocomp/software/tensorrt_demos')
from utils.camera import add_camera_args, Camera
from utils.display import open_window, set_display, show_fps
from utils.mtcnn import TrtMtcnn

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

_CONNECTIONS = [[15, 13], [13, 11], [16, 14], [14, 12], [11, 12], [5, 7], [6, 8], [7, 9], [8, 10],
                [1, 2], [0, 1], [0, 2], [1, 3], [2, 4], [3, 5], [4, 6], [17, 0], [17, 5], [17, 6],
                [17, 11], [17, 12]]

_JOINT_NAMES = ["nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder",
                "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist",
                "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle", "neck"]

_HEAD_PARTS = ["nose", "left_eye", "right_eye"]
               

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 1
        self.thread_period = 10
        if startup_check:
            self.startup_check()
        else:
            ########## user code ##################
            # config vars
            self.display = False
            self.with_objects = True
            self.human_parts_file = "human_pose.json"
            self.camera_name = "camera_top"
            self.min_number_of_joints = 3
            torch_model = 'densenet121_baseline_att_256x256_B_epoch_160.pth'
            optimized_model = 'densenet121_baseline_att_256x256_B_epoch_160_trt.pth'

            # camera
            try:
                rgbd = self.camerargbdsimple_proxy.getAll(self.camera_name)
                self.image_focalx = rgbd.image.focalx
                self.image_focaly = rgbd.image.focaly
                self.depth_focalx = rgbd.depth.focalx
                self.depth_focaly = rgbd.depth.focaly
                print("CameraRGBDSimple contacted. Image size rows:", rgbd.image.height, " cols:", rgbd.image.width)
            except:
                print("No connection to CameraRGBDSimple. Aborting")
                traceback.print_exc()
                sys.exit()

            # Hz
            self.cont = 0.01
            self.last_time = time.time()
            self.fps = 0

            # trt
            with open(self.human_parts_file, 'r') as f:
                self.human_pose = json.load(f)
            topology = trt_pose.coco.coco_category_to_topology(self.human_pose)
            self.parse_objects = ParseObjects(topology)
            self.TRT_MODEL_WIDTH = 256.
            self.TRT_MODEL_HEIGHT = 256.
            if os.path.exists(optimized_model) == False:
                print("Creating optimized model. Wait...")
                data = torch.zeros((1, 3, int(self.TRT_MODEL_HEIGHT), int(self.TRT_MODEL_WIDTH))).cuda()
                num_parts = len(self.human_pose['keypoints'])
                num_links = len(self.human_pose['skeleton'])
                model = trt_pose.models.densenet121_baseline_att(num_parts, 2 * num_links).cuda().eval()
                model.load_state_dict(torch.load(torch_model))
                self.model_trt = torch2trt.torch2trt(model, [data], fp16_mode=True, max_workspace_size=1 << 25)
                torch.save(self.model_trt.state_dict(), optimized_model)

            self.model_trt = TRTModule()
            self.model_trt.load_state_dict(torch.load(optimized_model))
            self.mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
            self.std = torch.Tensor([0.229, 0.224, 0.225]).cuda()

            # faces
            #self.mtcnn = TrtMtcnn()
            
            # camera read thread
            self.event = Event()
            self.read_image_queue = queue.Queue(1)
            self.read_camera_thread = Thread(target=self.get_rgb_thread, args=[self.camera_name, self.event],
                                      name="read_camera_queue")
            self.read_camera_thread.start()


            # result data
            self.people_data_write = ifaces.RoboCompHumanCameraBody.PeopleData()
            self.people_data_read = ifaces.RoboCompHumanCameraBody.PeopleData()

            #######################################

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        try:
            self.display = params["display"] == "true" or params["display"] == "True"
            self.with_objects = params["with_objects"] == "true" or params["with_objects"] == "True"
            if self.with_objects:  # objects read thread
                self.read_objects_queue = queue.Queue(1)
                self.read_objects_thread = Thread(target=self.get_objects_thread, args=[self.event],
                                          name="read_objects_queue", daemon=True)
                self.read_objects_thread.start()

            self.camera_name = params["camera_name"]
            self.min_number_of_joints = int(params["min_number_of_joints"])
            print("Params read. Starting...", params)
        except:
            print("Error reading config params")
            traceback.print_exc()
        return True

    @QtCore.Slot()
    def compute(self):
        color, depth, black, alive_time, period = self.read_image_queue.get()
        person_rois = []
        
        self.people_data_write = self.trtpose(black, depth, person_rois)  # get people from image
        
        # faces
        #dets, landmarks = self.mtcnn.detect(img, minsize=40)
        #print('{} face(s) found'.format(len(dets)))
        
        pad = 40
        for person in self.people_data_write.peoplelist:
            x_coor = []
            y_coor = []
            for k,v in person.joints.items():
                if k in _HEAD_PARTS:
                    x_coor.append(v.i)
                    y_coor.append(v.j)
            if x_coor and y_coor:
                r =  (min(x_coor), min(y_coor)), (max(x_coor), max(y_coor))
                cv2.rectangle(color, (r[0][0]-pad,r[0][1]-pad), (r[1][0]+pad, r[1][1]+pad), [255,0,0], 2)
                

        if self.display:
            self.show_data(color, self.people_data_write)
            
        self.people_data_write, self.people_data_read = self.people_data_read, self.people_data_write

        # FPS
        try:
            self.show_fps(alive_time, period)
        except KeyboardInterrupt:
            event.set()
            sys.exit()

    ####################################################################################################
    def get_rgb_thread(self, camera_name: str, event: Event):
        while not event.isSet():
            try:
                rgbd = self.camerargbdsimple_proxy.getAll(camera_name)
                color = np.frombuffer(rgbd.image.image, dtype=np.uint8).reshape(rgbd.image.height, rgbd.image.width, 3)
                depth = np.frombuffer(rgbd.depth.depth, dtype=np.float32).reshape(rgbd.depth.height, rgbd.depth.width, 1)
                black = color
                
                if self.with_objects:  # get person's rois from data.objects and copy on black image
                    data = self.read_objects_queue.get()
                    black, person_rois = self.get_person_rois(data, color)
     
                delta = int(1000 * time.time() - rgbd.image.alivetime)
                self.read_image_queue.put([color, depth, black, delta, rgbd.image.period])
                event.wait(self.thread_period / 1000)
            except:
                print("Error communicating with CameraRGBDSimple")
                traceback.print_exc()
                return

    def get_objects_thread(self, event: Event):
        while not event.isSet():
            try:
                data = self.yoloobjects_proxy.getYoloObjects()
                self.read_objects_queue.put(data)
                event.wait(self.thread_period / 1000)
            except:
                print("Error communicating with YoloObjects")
                traceback.print_exc()
                return

    def get_person_rois(self, data, rgb):  # reads objects from Yolo and computes masked image
        person_rois = []
        black = np.zeros(rgb.shape, dtype=np.uint8)
        for obj in data.objects:
            if obj.type == 0:  # person
                black[obj.top:obj.bot, obj.left: obj.right, :] = \
                    rgb[obj.top:obj.bot, obj.left: obj.right, :]
                person_rois.append([obj.left, obj.top, obj.right - obj.left, obj.bot - obj.top, obj.id])
        return black, person_rois

    def trtpose(self, rgb, depth, rois):
        # change to a padding
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
        center_x = rgb_cols / 2
        center_y = rgb_rows / 2
        people_data = ifaces.RoboCompHumanCameraBody.PeopleData(timestamp=time.time())
        people_data.peoplelist = []
        votes = np.zeros((counts[0], len(rois)), dtype=float)  # vote accumulator
        depth_to_rgb_factor = rgb_rows // depth.shape[0]
        for i in range(counts[0]):
            available_joints_counter = 0
            keypoints = self.get_keypoint(objects, i, peaks)
            joints = dict()
            for ids in range(len(keypoints)):
                if keypoints[ids][1] and keypoints[ids][2]:
                    available_joints_counter += 1
                    cx = int(keypoints[ids][2] * rgb_cols)
                    cy = int(keypoints[ids][1] * rgb_rows)
                    y = float(depth[cy // depth_to_rgb_factor, cx // depth_to_rgb_factor])
                    z = -(cy / self.depth_focalx) * (cy - center_y)
                    x = (cy / self.depth_focaly) * (cx - center_x)
                    joints[_JOINT_NAMES[ids]] = ifaces.RoboCompHumanCameraBody.KeyPoint(i=int(cx),
                                                                                        j=int(cy),
                                                                                        x=x, y=y, z=z)
                    if self.with_objects:
                        self.vote_for_roi(rois, (cx, cy), votes[i])
            if available_joints_counter < self.min_number_of_joints:
                continue
            
            for k,v in joints.items():
                if k in _HEAD_PARTS:
                    # build face's boundingbox
                    break
            
            people_data.peoplelist.append(ifaces.RoboCompHumanCameraBody.Person(id=i, joints=joints))

        # select the most voted ROI
        if self.with_objects and votes.size>0:
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
                peak = peaks[0][j][k]  # peak[1]:width, peak[0]:height
                peak = (j, float(peak[0]), float(peak[1]))
                kpoint.append(peak)
            else:
                peak = (j, None, None)
                kpoint.append(peak)
        return kpoint

    def show_data(self, rgb, people_data):
        for person in people_data.peoplelist:
            for cls, jnt in person.joints.items():
                cv2.circle(rgb, (jnt.i, jnt.j), 1, [0, 0, 255], 2)
        cv2.imshow('trtpose', rgb)
        cv2.waitKey(1)

    ####################################################################################################
    def show_fps(self, alive_time, period):
        if time.time() - self.last_time > 1:
            self.last_time = time.time()
            cur_period = int(1000. / self.cont)
            delta = (-1 if (period - cur_period) < -1 else (1 if (period - cur_period) > 1 else 0))
            print("Freq:", self.cont, "Hz. Alive_time:", alive_time, "ms. Img period:", int(period),
                  "ms. Curr period:", cur_period, "ms. Inc:", delta, "Timer:", self.thread_period)
            self.thread_period = np.clip(self.thread_period + delta, 0, 200)
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
        # img = self.write_image_queue.get()
        ret = ifaces.RoboCompCameraRGBDSimple.TImage()
        #     compressed=False,
        #     cameraID=0,
        #     height=img.shape[0],
        #     width=img.shape[1],
        #     depth=img.shape[2],
        #     focalx=self.image_focalx,
        #     focaly=self.image_focaly,
        #     alivetime=self.image_captured_time - time.time(),
        #     image=img.tobytes()
        # )
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


