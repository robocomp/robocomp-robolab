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

import sys, os, traceback, time

from PySide import QtGui, QtCore
from genericworker import *

import numpy as np
import pickle
import time
from visualizer import Visualizer
import time
import json

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel
class Timer:
	def __init__(self, fps):
		self.fps = fps
		self.ms = 1000 / fps
		self.tick = None

	# tock - current time in milliseconds
	def isReady(self, tock):
		if self.tick is None or tock - self.tick > self.ms:
			self.tick = tock
			return True
		return False

# order of joints to bring the results of the pose estimation component in compliance with CAD-60 joints order
# assumes that 8th joints is first replaced by the torso, which is calculated as mid-point between the neck and the middle of the hips
_joints_order_cad = [9, 8, 8, 13, 14, 12, 11, 3, 4, 2, 1, 15, 10, 5, 0]

current_milli_time = lambda: int(round(time.time() * 1000))

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 20
		self.timer.start(self.Period)
		self.visualizer = Visualizer()
		self.cam_timer = Timer(fps=30)
		self.pose_timer = Timer(fps=10)
		self.inference_timer = Timer(fps=10)
		self.print_timer = Timer(fps=0.5)

		self.camera_image = None
		self.img_restored = None
		self.skeleton2d = None
		self.skeleton3d = None

	def setParams(self, params):
		return True

	def adjustSkeleton(self, skeleton):
		# adjust the data to fit the activity recognition inference requirements
		skeleton = np.swapaxes(skeleton, 0, 1)
		skel_copy = np.copy(skeleton)
		torso = (skel_copy[:, 8] + skel_copy[:, 6]) / 2
		skeleton = skeleton[:, _joints_order_cad]
		skeleton[:, 2] = torso
		# pose estimator return values in meters, activity recognition expects values in mm
		skeleton = skeleton * 1000
		# in CAD-60 left and right sides are mirrored and z increases in depth direction
		skeleton[0, :] *= -1
		skeleton[1, :] *= -1
		skeleton[2, :] *= -1
		# skeleton[2, 3:] += 2000
		return skeleton


	@QtCore.Slot()
	def compute(self):
		now = current_milli_time()
		cam_ready = self.cam_timer.isReady(now)
		if cam_ready:
			try:
				self.camera_image = self.camerasimple_proxy.getImage()
				arr = np.fromstring(self.camera_image.image, np.uint8)
				self.img_restored = np.reshape(arr, (self.camera_image.width, self.camera_image.height, self.camera_image.depth))
			except Ice.Exception as e:
				traceback.print_exc()
				print(e)

		if self.pose_timer.isReady(now):
			skeleton2d, skeleton3d = self.poseestimation_proxy.getSkeleton(
				self.img_restored.data, [self.camera_image.width, self.camera_image.height, self.camera_image.depth])

			self.skeleton2d = np.asarray(skeleton2d)
			self.skeleton3d = np.asarray(skeleton3d, dtype=np.float32)
			self.skeleton3d = self.adjustSkeleton(skeleton3d)

		if cam_ready:
			self.visualizer.add_img(self.img_restored)
			if self.skeleton2d is not None:
				self.visualizer.add_point_2d(self.skeleton2d, (255, 0, 0))
			self.visualizer.show_all_imgs(pause=False)

		if self.inference_timer.isReady(now):
			ready = self.activityrecognition_proxy.addSkeleton(self.skeleton3d)
			if ready and self.print_timer.isReady(now):
				activity = self.activityrecognition_proxy.getCurrentActivity()
				print
				try:
					activity = json.loads(activity)
					for clazz, prob in sorted(activity.items(), key=lambda x: float(x[1]), reverse=True):
						print(clazz + ': ' + prob)
				except:
					print(activity)

		return True

