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

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from genericworker import *
import cv2
import numpy as np
import itertools

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 50
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        try:
            self.camera_name = params["camera_name"]
        except:
            print("Error reading config params")
        return True

    @QtCore.Slot()
    def compute(self):
        try:
            both = self.camerargbdsimple_proxy.getAll(self.camera_name)
            color = both.image
            depth = both.depth
            cvdepth = np.frombuffer(depth.depth, dtype=np.float32).reshape(depth.height, depth.width)
            cvdepth_norm = cv2.normalize(src=cvdepth, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            cvdepth_norm_color = cv2.applyColorMap(cvdepth_norm, cv2.COLORMAP_HOT)
            cvcolor = np.frombuffer(color.image, np.uint8).reshape(color.height, color.width, color.depth)
            mosaic = self.make_mosaic([cvcolor, cvdepth_norm_color])
            cv2.imshow('CameraRGBDViewer', mosaic)
            #cv2.imshow('CameraRGBDViewer - Depth', cvdepth_norm)
        except Ice.Exception as e:
            print(e)
        return True

    def make_mosaic(self, imgs):
        #if any(i.shape != imgs[0].shape for i in imgs[1:]):
        #    raise ValueError('Not all images have the same shape.')

        img_h, img_w, img_c = imgs[0].shape
        m_x = 0
        m_y = 0
        w = 2
        h = 1
        n = w * h
        margin = 10
        if margin is not None:
            m_x = int(margin)
            m_y = m_x
        imgmatrix = np.zeros((img_h * h + m_y * (h - 1),
                              img_w * w + m_x * (w - 1),
                              img_c),
                             np.uint8)
        imgmatrix.fill(255)
        positions = itertools.product(range(w), range(h))
        for (x_i, y_i), img in itertools.zip_longest(positions, imgs):
            x = x_i * (img_w + m_x)
            y = y_i * (img_h + m_y)
            imgmatrix[y:y+img_h, x:x+img_w, :] = img

        return imgmatrix

def startup_check(self):
    QTimer.singleShot(200, QApplication.instance().quit)

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

#  def compute(self):
# 		try:
# 			both = self.camerargbdsimple_proxy.getAll(
# 				"Viriato_head_camera_front_sensor")
# 			color = both.image
# 			depth = both.depth
# 			cvdepth = np.frombuffer(depth.depth, dtype=np.float32).reshape(
# 				depth.height, depth.width)
# 			cvcolor = np.frombuffer(color.image, np.uint8).reshape(
# 				color.height, color.width, color.depth)
# 			cv2.imshow('CameraRGBDViewer', cvcolor)
# 		except Ice.Exception as e:
# 			print(e)
#         return True
