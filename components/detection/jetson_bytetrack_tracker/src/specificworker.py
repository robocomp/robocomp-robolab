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
import queue
import itertools
import traceback
from threading import Thread
import os
import os.path as osp
import time
import cv2
import torch
from loguru import logger
import numpy as np

sys.path.append('/home/robocomp/software/ByteTrack')
from yolox.data.data_augment import preproc
from yolox.exp import get_exp
from yolox.utils import fuse_model, get_model_info, postprocess
from yolox.utils.visualize import plot_tracking
from yolox.tracker.byte_tracker import BYTETracker
from yolox.tracking_utils.timer import Timer

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

_OBJECT_NAMES = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
                 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse',
                 'sheep',
                 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase',
                 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
                 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
                 'banana', 'apple',
                 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
                 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard',
                 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
                 'scissors',
                 'teddy bear', 'hair drier', 'toothbrush']

class Predictor(object):
    def __init__(
        self,
        model,
        exp,
        trt_file=None,
        decoder=None,
        device=torch.device("cuda"),
        fp16=False
    ):
        self.model = model
        self.decoder = decoder
        self.num_classes = exp.num_classes
        self.confthre = exp.test_conf
        self.nmsthre = exp.nmsthre
        self.test_size = exp.test_size
        self.device = device
        self.fp16 = fp16
        if trt_file is not None:
            from torch2trt import TRTModule
            model_trt = TRTModule()
            model_trt.load_state_dict(torch.load(trt_file))
            x = torch.ones((1, 3, exp.test_size[0], exp.test_size[1]), device=device)
            self.model(x)
            self.model = model_trt
        self.rgb_means = (0.485, 0.456, 0.406)
        self.std = (0.229, 0.224, 0.225)

    def inference(self, img, timer):
        img_info = {"id": 0}
        height, width = img.shape[:2]
        img_info["height"] = height
        img_info["width"] = width
        img_info["raw_img"] = img

        t0 = time.time()
        img, ratio = preproc(img, self.test_size, self.rgb_means, self.std)
        t1 = time.time()
        img_info["ratio"] = ratio
        img = torch.from_numpy(img).unsqueeze(0).float().to(self.device)
        if self.fp16:
            img = img.half()  # to FP16
        t2 = time.time()
        with torch.no_grad():
            timer.tic()
            outputs = self.model(img)
            t3 = time.time()
            if self.decoder is not None:
                outputs = self.decoder(outputs, dtype=outputs.type())
            outputs = postprocess(outputs, self.num_classes, self.confthre, self.nmsthre)
            t4 = time.time()
        print(1000.0*(t1-t0), 1000.0*(t2-t1), 1000.0*(t3-t2), 1000.0*(t4-t3))
        return outputs, img_info


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 1
        if startup_check:
            self.startup_check()
        else:

            self.time_init = time.time()

            self.exp = get_exp('/home/robocomp/software/ByteTrack/exps/example/mot/yolox_s_mix_det.py', "")
            device = torch.device("cuda")
            model = self.exp.get_model().to(device)
            print("Model Summary: {}".format(get_model_info(model, self.exp.test_size)))
            model.eval()
            trt_file = "/home/nvidia/software/ByteTrack/YOLOX_outputs/yolox_s_mix_det/model_trt.pth"
            assert osp.exists(trt_file), "TensorRT model is not found!\n Run python3 tools/trt.py first!"
            model.head.decode_in_inference = False
            decoder = model.head.decode_outputs
            print("Using TensorRT to inference")
            self.predictor = Predictor(model, self.exp, trt_file, decoder, device, False)
            self.tracker = BYTETracker(frame_rate=60)

            # camera read thread
            self.read_queue = queue.Queue(1)
            self.read_thread = Thread(target=self.get_rgb_thread, args=["camera_top"], name="read_queue", daemon=True)
            self.read_thread.start()

            # result data
            self.objects_write = ifaces.RoboCompYoloObjects.TData()
            self.objects_read = ifaces.RoboCompYoloObjects.TData()

            # Hz
            self.cont = 0
            self.last_time = time.time()
            self.fps = 0

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        try:
            self.display = params["display"] == "true" or params["display"] == "True"
        except:
            traceback.print_exc()
            print("Error reading config params")
        print("Config params: ", params)
        return True


    @QtCore.Slot()
    def compute(self):
        self.time_init = time.time()
        frame = self.read_queue.get()
        t1 = time.time()
        outputs, img_info = self.predictor.inference(frame, Timer())
        t2 = time.time()
        self.objects_write = self.post_process(outputs, img_info)
        #print(1000.0*(t1-self.time_init), 1000.0*(t2-t1), 1000.0*(time.time()-t2))
        #
        # if self.display:
        #     self.display_data(outputs, frame, img_info)
        #
        # self.objects_write, self.objects_read = self.objects_read, self.objects_write

        # FPS
        self.show_fps()

    ##########################################################################################
    def get_rgb_thread(self, camera_name: str):
        while True:
            try:
                rgb = self.camerargbdsimple_proxy.getImage(camera_name)
                frame = np.frombuffer(rgb.image, dtype=np.uint8)
                frame = frame.reshape((rgb.height, rgb.width, 3))
                self.read_queue.put(frame)
            except:
                print("Error communicating with CameraRGBDSimple")
                traceback.print_exc()
                return

    def post_process(self, outputs, img_info):
        if outputs[0] is not None:
            data = ifaces.RoboCompYoloObjects.TData()
            data.objects = []
            data.people = []
            online_targets = self.tracker.update(outputs[0], [img_info['height'], img_info['width']], self.exp.test_size)
            for t in online_targets:
                tlwh = t.tlwh
                tid = t.track_id
                vertical = tlwh[2] / tlwh[3] > 1.6
                if tlwh[2] * tlwh[3] > 10 and not vertical:
                    ibox = ifaces.RoboCompYoloObjects.TBox()
                    ibox.type = 0
                    ibox.id = tid
                    ibox.score = t.score
                    ibox.left = int(tlwh[0])
                    ibox.top = int(tlwh[1])
                    ibox.right = int(tlwh[0]+tlwh[2])
                    ibox.bot = int(tlwh[1]+tlwh[3])
                    data.objects.append(ibox)
        return data

    def display_data(self, outputs, frame, img_info):
        frame_id = 0
        online_tlwhs = []
        online_ids = []
        online_scores = []
        if outputs[0] is not None:
            online_targets = self.tracker.update(outputs[0], [img_info['height'], img_info['width']], self.exp.test_size)
            for t in online_targets:
                tlwh = t.tlwh
                tid = t.track_id
                vertical = tlwh[2] / tlwh[3] > 1.6
                if tlwh[2] * tlwh[3] > 10 and not vertical:
                    online_tlwhs.append(tlwh)
                    online_ids.append(tid)
                    online_scores.append(t.score)
            frame = plot_tracking(img_info["raw_img"], online_tlwhs, online_ids, frame_id=frame_id + 1,
                                  fps=1./(time.time()-self.time_init))
        else:
            timer.toc()
        cv2.imshow('ByteTrack', frame)
        cv2.waitKey(1)

    def show_fps(self):
        if time.time() - self.last_time > 1:
            self.last_time = time.time()
            print("Freq: ", self.cont, "Hz. Waiting for image")
            self.cont = 0
        else:
            self.cont += 1

    ###########################################################################################
    def startup_check(self):
        print(f"Testing RoboCompCameraRGBDSimple.TImage from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TImage()
        print(f"Testing RoboCompCameraRGBDSimple.TDepth from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TDepth()
        print(f"Testing RoboCompCameraRGBDSimple.TRGBD from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TRGBD()
        print(f"Testing RoboCompYoloObjects.TBox from ifaces.RoboCompYoloObjects")
        test = ifaces.RoboCompYoloObjects.TBox()
        print(f"Testing RoboCompYoloObjects.TKeyPoint from ifaces.RoboCompYoloObjects")
        test = ifaces.RoboCompYoloObjects.TKeyPoint()
        print(f"Testing RoboCompYoloObjects.TPerson from ifaces.RoboCompYoloObjects")
        test = ifaces.RoboCompYoloObjects.TPerson()
        print(f"Testing RoboCompYoloObjects.TConnection from ifaces.RoboCompYoloObjects")
        test = ifaces.RoboCompYoloObjects.TConnection()
        print(f"Testing RoboCompYoloObjects.TJointData from ifaces.RoboCompYoloObjects")
        test = ifaces.RoboCompYoloObjects.TJointData()
        print(f"Testing RoboCompYoloObjects.TData from ifaces.RoboCompYoloObjects")
        test = ifaces.RoboCompYoloObjects.TData()
        QTimer.singleShot(200, QApplication.instance().quit)



    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getImage method from YoloObjects interface
    #
    def YoloObjects_getImage(self):
        ret = ifaces.RoboCompYoloObjects.RoboCompCameraRGBDSimple.TImage()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of getYoloJointData method from YoloObjects interface
    #
    def YoloObjects_getYoloJointData(self):
        ret = ifaces.RoboCompYoloObjects.TJointData()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of getYoloObjectNames method from YoloObjects interface
    #
    def YoloObjects_getYoloObjectNames(self):
        return _OBJECT_NAMES
    #
    # IMPLEMENTATION of getYoloObjects method from YoloObjects interface
    #
    def YoloObjects_getYoloObjects(self):
        return self.objects_read
    # ===================================================================
    # ===================================================================


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
    # From the RoboCompYoloObjects you can use this types:
    # RoboCompYoloObjects.TBox
    # RoboCompYoloObjects.TKeyPoint
    # RoboCompYoloObjects.TPerson
    # RoboCompYoloObjects.TConnection
    # RoboCompYoloObjects.TJointData
    # RoboCompYoloObjects.TData


