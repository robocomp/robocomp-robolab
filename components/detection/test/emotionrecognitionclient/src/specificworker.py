#
# Copyright (C) 2018 by YOUR NAME HERE
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
import numpy as np
import cv2
from PySide import QtGui, QtCore
from genericworker import *

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 100
		self.timer.start(self.Period)

	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		print('SpecificWorker.compute...')

		# Get image from camera
		data = self.camerasimple_proxy.getImage()
		arr = np.fromstring(data.image, np.uint8)
		frame = np.reshape(arr,(data.height, data.width, data.depth))

		# Get emotion list
		emotionL = self.emotionrecognition_proxy.getEmotionList()
		print(emotionL)

		# Showing data on the frame
		for emotionData in emotionL:
			x = emotionData.x
			y = emotionData.y
			w = emotionData.w
			h = emotionData.h
			emotion = emotionData.emotion
			cv2.rectangle(frame, (x,y), (x+w,y+h), (255,0,0),2)
			cv2.putText(frame, emotion, (x,y-2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255) ,2 , cv2.LINE_AA)
		cv2.imshow('Emotion', frame)

		return True

