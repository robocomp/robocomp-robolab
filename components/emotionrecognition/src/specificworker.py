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
import tensorflow as tf
from tensorflow.python.platform import gfile
from PySide import QtGui, QtCore
from genericworker import *

MODEL_FILE = 'assets/emotion_classifier_dsift.pb'
IMAGE_SIZE = 48
NUM_CHANNELS = 1
NUM_SIFT = 2048
EMOTIONS=['Happy','Sad','Neutral','Angry','Scared','Surprised']

face_cascade = cv2.CascadeClassifier('assets/haarcascade_frontalface_default.xml')
mean_sift=np.load("assets/mean_sift.npy")
std_sift=np.load("assets/std_sift.npy")

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 100
		self.timer.start(self.Period)

		# SIFT feature extractor
		self.feature_extractor = cv2.xfeatures2d.SIFT_create()

		# Create a dense grid of keypoints
		self.keypoints=list()
		for i in range(5,IMAGE_SIZE,12):
			for j in range(5,IMAGE_SIZE,12):
				self.keypoints.append(cv2.KeyPoint(i,j,12))

		# Create a tensorflow session
		self.sess=tf.Session()

		# Read the frozen graph from the model file
		with gfile.FastGFile(MODEL_FILE,'rb') as f:
			graph_def = tf.GraphDef()
			graph_def.ParseFromString(f.read())
			self.sess.graph.as_default()
			tf.import_graph_def(graph_def, name='')

			# Get input and output tensors from graph
			self.x_input = self.sess.graph.get_tensor_by_name("input:0")
			self.output = self.sess.graph.get_tensor_by_name("output:0")
			self.dsift = self.sess.graph.get_tensor_by_name("sift:0")


	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'
		try:
			data = self.camerasimple_proxy.getImage()
			arr = np.fromstring(data.image, np.uint8)
			frame = np.reshape(arr, (data.width, data.height, data.depth))

			# Convert frame to grayscale for recognition
			gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY )

			# Detect faces
			faces = face_cascade.detectMultiScale(gray)

			emotions_temp = list()
			for (x,y,w,h) in faces:
				# Crop out the face and extract D-SIFT features
				cropped_frame = gray[y:y+h, x:x+w]
				cropped_frame = cv2.resize(cropped_frame, (IMAGE_SIZE,IMAGE_SIZE))
				_, features = self.feature_extractor.compute(cropped_frame,self.keypoints)
				features = (features-mean_sift)/std_sift
				features = features.reshape(1,NUM_SIFT)

				# Do necessary preprocessing
				cropped_frame = cropped_frame.reshape((1,IMAGE_SIZE,IMAGE_SIZE,NUM_CHANNELS))
				cropped_frame = (cropped_frame-np.mean(cropped_frame))/np.std(cropped_frame)

				# Feed the cropped and preprocessed frame to classifier
				result = self.sess.run(self.output, {self.x_input:cropped_frame, self.dsift:features})

				# Get the emotion
				emotion = EMOTIONS[np.argmax(result)]

				# Store emotion data
				emotionData = SEmotion()
				emotionData.x = x
				emotionData.y = y
				emotionData.w = w
				emotionData.h = h
				emotionData.emotion = emotion
				emotions_temp.append(emotionData)

			self.emotionList = emotions_temp

		except Ice.Exception, e:
			traceback.print_exc()
			print e

		cv2.waitKey(1)
		return True

	def getEmotionList(self):
		return self.emotionList