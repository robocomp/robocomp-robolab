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
import itertools

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import cv2
import traceback
import numpy as np
import time
from numpy.random import choice as CColor # ChooseColor
from shapely.geometry import box
from collections import defaultdict

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

_COLORS = np.array(
    [
        0.000, 0.447, 0.741,
        0.850, 0.325, 0.098,
        0.929, 0.694, 0.125,
        0.494, 0.184, 0.556,
        0.466, 0.674, 0.188,
        0.301, 0.745, 0.933,
        0.635, 0.078, 0.184,
        0.300, 0.300, 0.300,
        0.600, 0.600, 0.600,
        1.000, 0.000, 0.000,
        1.000, 0.500, 0.000,
        0.749, 0.749, 0.000,
        0.000, 1.000, 0.000,
        0.000, 0.000, 1.000,
        0.667, 0.000, 1.000,
        0.333, 0.333, 0.000,
        0.333, 0.667, 0.000,
        0.333, 1.000, 0.000,
        0.667, 0.333, 0.000,
        0.667, 0.667, 0.000,
        0.667, 1.000, 0.000,
        1.000, 0.333, 0.000,
        1.000, 0.667, 0.000,
        1.000, 1.000, 0.000,
        0.000, 0.333, 0.500,
        0.000, 0.667, 0.500,
        0.000, 1.000, 0.500,
        0.333, 0.000, 0.500,
        0.333, 0.333, 0.500,
        0.333, 0.667, 0.500,
        0.333, 1.000, 0.500,
        0.667, 0.000, 0.500,
        0.667, 0.333, 0.500,
        0.667, 0.667, 0.500,
        0.667, 1.000, 0.500,
        1.000, 0.000, 0.500,
        1.000, 0.333, 0.500,
        1.000, 0.667, 0.500,
        1.000, 1.000, 0.500,
        0.000, 0.333, 1.000,
        0.000, 0.667, 1.000,
        0.000, 1.000, 1.000,
        0.333, 0.000, 1.000,
        0.333, 0.333, 1.000,
        0.333, 0.667, 1.000,
        0.333, 1.000, 1.000,
        0.667, 0.000, 1.000,
        0.667, 0.333, 1.000,
        0.667, 0.667, 1.000,
        0.667, 1.000, 1.000,
        1.000, 0.000, 1.000,
        1.000, 0.333, 1.000,
        1.000, 0.667, 1.000,
        0.333, 0.000, 0.000,
        0.500, 0.000, 0.000,
        0.667, 0.000, 0.000,
        0.833, 0.000, 0.000,
        1.000, 0.000, 0.000,
        0.000, 0.167, 0.000,
        0.000, 0.333, 0.000,
        0.000, 0.500, 0.000,
        0.000, 0.667, 0.000,
        0.000, 0.833, 0.000,
        0.000, 1.000, 0.000,
        0.000, 0.000, 0.167,
        0.000, 0.000, 0.333,
        0.000, 0.000, 0.500,
        0.000, 0.000, 0.667,
        0.000, 0.000, 0.833,
        0.000, 0.000, 1.000,
        0.000, 0.000, 0.000,
        0.143, 0.143, 0.143,
        0.286, 0.286, 0.286,
        0.429, 0.429, 0.429,
        0.571, 0.571, 0.571,
        0.714, 0.714, 0.714,
        0.857, 0.857, 0.857,
        0.000, 0.447, 0.741,
        0.314, 0.717, 0.741,
        0.50, 0.5, 0
    ]
).astype(np.float32).reshape(-1, 3)

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 33
        if startup_check:
            self.startup_check()
        else:
            ########## user code ##################
            # Hz
            self.cont = 0
            self.last_time = time.time()

            try:
                self.joint_data = self.humancamerabody_proxy.getJointData()
            except Ice.Exception:
                traceback.print_exc()
                print("Error connecting with HumanCameraBody to read TJointData")
                sys.exit()
            print("Connected to HumanCameraBody")
            print("Joint names: ", self.joint_data.jointNames)
            #print("Connections:", self.joint_data.connections)

            try:
                self.object_names = self.yoloobjects_proxy.getYoloObjectNames()
                print("Connected to YoloObjects")
                print(self.object_names)

            except Ice.Exception as e:
                traceback.print_exc()

            ######################################

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        return True


    @QtCore.Slot()
    def compute(self):
        try:
            img = self.camerargbdsimple_proxy.getImage('camera_top')
            color = np.frombuffer(img.image, dtype=np.uint8)
            color = color.reshape((img.height, img.width, 3))
        except Ice.Exception as e:
            traceback.print_exc()

        try:
            people = self.humancamerabody_proxy.newPeopleData()
            self.draw_people(people, color)

        except Ice.Exception as e:
            traceback.print_exc()

        try:
            data = self.yoloobjects_proxy.getYoloObjects()
            nms = self.nms(data.objects)
            #print("lens", len(data.objects), len(nms))
            self.draw_objects(nms, people, color)
        except Ice.Exception as e:
            traceback.print_exc()

        cv2.imshow("Jetson", color)
        cv2.waitKey(1)  # 1 millisecond

        # FPS
        self.show_fps()
        return True

    ##############################################################################################
    # Non-maximum supression
    def nms(self, objects):
        d = defaultdict(list)
        for obj in objects:
            d[obj.type].append(obj)
        removed = []
        for typ, same_type_objs in d.items():
            power_set = itertools.combinations(same_type_objs, 2)   # possible combs of same type
            # compute IOU
            for a, b in power_set:
                p1 = box(a.left, a.top, a.right, a.bot)  # shapely object
                p2 = box(b.left, b.top, b.right, b.bot)
                intersect = p1.intersection(p2).area / p1.union(p2).area
                if intersect > 0.65:
                    removed.append(a.id if abs(a.prob) > abs(b.prob) else b.id)
        ret = []
        for obj in objects:
            if obj.id not in removed:
                ret.append(obj)
        return ret

    def draw_objects(self, objects, people, img):
        for box in objects:
            tl = round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
            color = (_COLORS[box.type] * 255).astype(np.uint8).tolist()
            c1 = (box.left, box.top)
            c2 = (box.right, box.bot)
            cv2.rectangle(img, c1, c2, color, tl, cv2.LINE_AA)

            txt_color = (0, 0, 0) if np.mean(_COLORS[box.type]) > 0.5 else (255, 255, 255)
            txt_bk_color = (_COLORS[box.type] * 255 * 0.7).astype(np.uint8).tolist()
            tf = int(max(tl - 1, 1))
            if box.type == 0:
                hid = -1
                #print("box", box.id)
                for person in people.peoplelist:
                    #print("person", person.id)
                    if person.id == box.id:
                        hid = box.id
                        break
                #print("------------------")
                label = self.object_names[box.type] + "_" + str(box.id) + "_H" + str(hid) + " " + str(int((-box.prob * 100))) + '%'
            else:
               label = self.object_names[box.type] + "_" + str(box.id) + " " + str(int((box.prob * 100))) + '%'

            t_size, _ = cv2.getTextSize(label, 0, tl/3.0, tf)
            c2 = (c1[0] + t_size[0], c1[1] - t_size[1] - 3)
            cv2.rectangle(img, c1, c2, txt_bk_color, -1, cv2.LINE_AA)  #filled
            cv2.putText(img, label, (c1[0], c1[1]-2), 0, tl/3, txt_color, tf, cv2.LINE_AA)

    def draw_people(self, people, img: np.ndarray):
        for person in people.peoplelist:
            for conn in self.joint_data.connections:
                try:
                    a = self.joint_data.jointNames[conn.first]
                    b = self.joint_data.jointNames[conn.second]
                    if (a in person.joints and b in person.joints):
                        cv2.line(img, (person.joints[a].i, person.joints[a].j),
                                 (person.joints[b].i, person.joints[b].j),
                                 (0, 128, 0), 4)
                except:
                    pass

            for id, jnt in person.joints.items():
                cv2.circle(img, (jnt.i, jnt.j), 3, (255, 255, 255), 2)

    ##############################################################################################
    def startup_check(self):
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

    def show_fps(self):
        if time.time() - self.last_time > 1:
            self.last_time = time.time()
            print("Freq: ", self.cont, "Hz. Waiting for image")
            self.cont = 0
        else:
            self.cont += 1
    ######################
    # From the RoboCompCameraRGBDSimple you can call this methods:
    # self.camerargbdsimple_proxy.getAll(...)
    # self.camerargbdsimple_proxy.getDepth(...)
    # self.camerargbdsimple_proxy.getImage(...)

    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # RoboCompCameraRGBDSimple.TImage
    # RoboCompCameraRGBDSimple.TDepth
    # RoboCompCameraRGBDSimple.TRGBD

    ######################
    # From the RoboCompHumanCameraBody you can call this methods:
    # self.humancamerabody_proxy.newPeopleData(...)

    ######################
    # From the RoboCompHumanCameraBody you can use this types:
    # RoboCompHumanCameraBody.TImage
    # RoboCompHumanCameraBody.TGroundTruth
    # RoboCompHumanCameraBody.KeyPoint
    # RoboCompHumanCameraBody.Person
    # RoboCompHumanCameraBody.PeopleData


