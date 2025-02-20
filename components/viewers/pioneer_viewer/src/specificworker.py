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

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from genericworker import *
import cv2
import numpy as np
import itertools

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 5
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

        # QLabel to show the image
        self.label = QLabel(self.ui.frame)
        self.label.setGeometry(0, 0, self.ui.frame.width(), self.ui.frame.height())
        self.label.setAlignment(Qt.AlignCenter)  # Alinear la imagen al centro

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
            #cvdepth = np.frombuffer(depth.depth, dtype=np.float32).reshape(depth.height, depth.width)
            #cvdepth_norm = cv2.normalize(src=cvdepth, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            #cvdepth_norm_color = cv2.applyColorMap(cvdepth_norm, cv2.COLORMAP_HOT)
            cvcolor = np.frombuffer(color.image, np.uint8).reshape(color.height, color.width, color.depth)
            #cvcolor = cv2.resize(cvcolor, (color.height*2, color.width*2))
            # Verifica si la imagen se cargó correctamente
            print("Imagen cargada, dimensiones: ", cvcolor.shape)
            #cv2.imshow("Debug - Imagen recibida", cvcolor)  # Mostrar la imagen con OpenCV para depurar

            #Convert the OpenCV image to QImage
            image = cv2.cvtColor(cvcolor, cv2.COLOR_BGR2RGB)
            # Verificar las dimensiones y tipo de imagen
            print("Dimensiones de la imagen convertida: ", image.shape)
            if image is not None:
                # Convertir la imagen de OpenCV a QImage
                height, width, channels = image.shape
                qimage = QImage(image.data, width, height, width * channels, QImage.Format_RGB888)

                # Convertir QImage a QPixmap y actualizar el QLabel
                pixmap = QPixmap.fromImage(qimage)
                self.label.setPixmap(pixmap)

                # Ajustar la escala del contenido del QLabel
                self.label.setScaledContents(True)
                self.ui.frame.show()  # Mostrar el frame

        except Ice.Exception as e:
            print(e)


        #self.ui.progressBar.setValue(0)

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
