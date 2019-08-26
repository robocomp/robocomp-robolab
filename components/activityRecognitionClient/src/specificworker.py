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

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


# test sample path, used only in debug mode
_test_sample_path = 'src/data/test_sample.npy'
# turn on the debug mode to test the client + activity recognition component without the real camera input
_DEBUG = False
# do pose estimation at 10 fps
_wait_time_pose = 1000/10
# do activity recognition inference at 1 fps
_wait_time_inference = 1000/1
# order of joints to bring the results of the pose estimation component in compliance with CAD-60 joints order
# assumes that 8th joints is first replaced by the torso, which is calculated as mid-point between the neck and the middle of the hips
_joints_order_cad = [9, 8, 8, 13, 14, 12, 11, 3, 4, 2, 1, 15, 10, 5, 0]

current_milli_time = lambda: int(round(time.time() * 1000))

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
		self.prev = current_milli_time() - 100
		self.prev2 = current_milli_time() - 1000
		self.visualizer = Visualizer()

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
		skeleton[2, 3:] += 2000
		return skeleton


	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'
		if _DEBUG:
			print('Component is run in the debug mode on one test sample, change the _DEBUG variable in the specificworker.py to False to run in the normal mode')
			sample = np.load(_test_sample_path)
			for i in range(sample.shape[1]):
				ready = self.activityrecognition_proxy.addSkeleton(sample[:, i, :])
				if ready:
					activity= self.activityrecognition_proxy.getCurrentActivity()
					print(activity)
		else:
			now = current_milli_time()
			if now - self.prev >= _wait_time_pose:
				self.prev = now
				try:
					data = self.camerasimple_proxy.getImage()
				except Ice.Exception, e:
					traceback.print_exc()
					print e
				# reconstruct a 3d array from the input. (input comes as a string)
				arr = np.fromstring(data.image, np.uint8)
				img_restored = np.reshape(arr, (data.width, data.height, data.depth))

				skeleton2d, skeleton3d = self.poseestimation_proxy.getSkeleton(img_restored.data, [data.width, data.height, data.depth])

				skeleton2d = np.asarray(skeleton2d)
				skeleton3d = np.asarray(skeleton3d, dtype=np.float32)	
				skeleton3d = self.adjustSkeleton(skeleton3d)
				
				self.visualizer.add_img(img_restored)
				self.visualizer.add_point_2d(skeleton2d, (255, 0, 0))
				self.visualizer.show_all_imgs(pause=False)
				now2 = current_milli_time()
				if now2 - self.prev2 >= _wait_time_inference:
					self.prev2 = now2
					ready = self.activityrecognition_proxy.addSkeleton(skeleton3d)
					if ready:
						activity= self.activityrecognition_proxy.getCurrentActivity()
						print(activity)


		return True

