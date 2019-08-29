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

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

import collections
import numpy as np
import pickle
from inference import Predictor
import json


_class_names = ['talking on the phone', 'writing on whiteboard', 'drinking water', 'rinsing mouth with water', 'brushing teeth', 'wearing contact lenses',
 'talking on couch', 'relaxing on couch', 'cooking (chopping)', 'cooking (stirring)', 'opening pill container', 'working on computer']

def predictionToJson(classes, probs):
	d = {}
	for c, p in zip(classes, probs):
		d[_class_names[c]] = str(p)
	return json.dumps(d)


class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)

		# number of frames necessary to run the prediction
		self.min_num_frames = 32
		# stores the latest predicted acivity
		self.currentActivity = None
		# 4 latest poses required for inference
		self.skeletons = collections.deque()
		# load the model
		self.predictor = Predictor()

	# indicator that component has received at least n frames and made a prediction
	@property
	def ready(self):
		return len(self.skeletons) >= self.min_num_frames

	def predict(self):
		# extract features from the existing skeleton frames, run prediction and print the top 5 predictions
		sample = np.array(self.skeletons)
		sample = np.swapaxes(sample, 0, 1)
		classes, probs = self.predictor.estimate(sample)
		self.currentActivity = predictionToJson(classes, probs)
		# self.currentActivity = _class_names[classes[0]]

	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		return True


	#
	# getCurrentActivity
	#
	def getCurrentActivity(self):
		return self.currentActivity


	#
	# addSkeleton
	#
	def addSkeleton(self, skeleton):
		self.skeletons.append(skeleton)
		if len(self.skeletons) > self.min_num_frames:
			self.skeletons.popleft()
		if len(self.skeletons) == self.min_num_frames:
			self.predict()
		return self.ready

