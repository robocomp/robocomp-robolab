#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
# Copyright (C) 2020 by YOUR NAME HERE
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
import sys, os, traceback, time
import numpy as np
import cv2
sys.path.append(os.path.join(os.getcwd(),"assets"))
from darknet import *

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 50
		self.timer.start(self.Period)
		self.img_name = './temp.jpg'


	def __del__(self):
		os.system('rm -rf %s'%(self.img_name))
		print('SpecificWorker destructor')

	def setParams(self, params):
		#try:
		#	self.innermodel = InnerModel(params["InnerModelPath"])
		#except:
		#	traceback.print_exc()
		#	print("Error reading config params")
		return True

	@QtCore.Slot()
	def compute(self):
		print('SpecificWorker.compute...')
		data = self.camerasimple_proxy.getImage()
		arr = np.fromstring(data.image, np.uint8)
		frame = np.reshape(arr, (data.height, data.width, data.depth))
		cv2.imwrite(self.img_name, frame)
		res = performDetect(self.img_name, configPath="./assets/cfg/yolov4.cfg", weightPath = "./assets/yolov4.weights", metaPath= "./assets/cfg/coco.data", showImage= False)
		os.system('rm -rf %s'%(self.img_name))
		for i in range(len(res)):
			if (res[i][1] >= 0.3):
				cv2.rectangle(frame, (int(res[i][2][0] - res[i][2][2]/2),int(res[i][2][1] - res[i][2][3]/2)), (int(res[i][2][0] + res[i][2][2]/2),int(res[i][2][1] + res[i][2][3]/2)), (255,0,0),2)
				cv2.putText(frame, res[i][0], (int(res[i][2][0] - res[i][2][2]/2),int(res[i][2][1] - 2 - res[i][2][3]/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1 , cv2.LINE_AA)
		cv2.imshow('Output', frame)	
		cv2.waitKey(1)
		return True

