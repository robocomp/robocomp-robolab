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
import imutils
import dlib
import math
from numpy import linalg as LA
from imutils import face_utils
from tensorflow.python.platform import gfile
from PySide import QtGui, QtCore
from genericworker import *
from face_alignment import FaceAligner
import face_detector

MODEL_FILE = 'assets/emotion_classifier_fl.pb'
IMAGE_SIZE = 128
NUM_CHANNELS = 1
NUM_FL = 272
EMOTIONS=['Happy','Sad','Neutral','Angry','Surprised']

mean_fl=np.load("assets/mean_fl.npy")
std_fl=np.load("assets/std_fl.npy")
predictor = dlib.shape_predictor("assets/shape_predictor_68_face_landmarks.dat")

features = np.empty([1, NUM_FL], dtype=np.float64)
class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 100
		self.timer.start(self.Period)

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
			self.fl = self.sess.graph.get_tensor_by_name("fl:0")


	def getFL(self, image):
		rect=dlib.rectangle( 0, 0, IMAGE_SIZE, IMAGE_SIZE )
		landmarks = predictor(image, rect)
		landmarks = face_utils.shape_to_np(landmarks)

		# For testing purpose
		image_copy = np.copy(image)
		for (x, y) in landmarks:
			cv2.circle(image_copy, (x, y), 1, (0, 0, 255), -1)
			cv2.imshow("Facial Landmarks", image_copy)

		center = np.mean(landmarks, axis=0)
		dist = LA.norm(landmarks - center, axis=1)
		angles = (np.arctan2(landmarks[:,0], landmarks[:,1])*360)/(2*math.pi)

		for i in range(68):
			features[0,i*4] = landmarks[i,0]
			features[0,i*4+1] = landmarks[i,1]
			features[0,i*4+2] = dist[i]
			features[0,i*4+3] = angles[i]

		return features

	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'
		try:
			data = self.camerasimple_proxy.getImage()
			arr = np.fromstring(data.image, np.uint8)
			frame = np.reshape(arr, (data.width, data.height, data.depth))
			gray=cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY )

			# Detect faces
			faces = face_detector.detect(frame)

			emotions_temp = list()
			for (x1,y1,x2,y2) in faces:

				# Align the face
				fa = FaceAligner(predictor,desiredFaceWidth=IMAGE_SIZE*3)
				faceAligned = fa.align(frame, gray, dlib.rectangle(x1,y1,x2,y2))

				# Closely crop out the face
				faces2 = face_detector.detect(faceAligned)
				(xn1,yn1,xn2,yn2) = faces2[0]
				cropped_frame = faceAligned[yn1:yn2, xn1:xn2]

				# Convert to grayscale and apply adaptive histogram equalization
				cropped_frame = cv2.cvtColor(cropped_frame,cv2.COLOR_RGB2GRAY )
				clahe = cv2.createCLAHE(clipLimit=10.0, tileGridSize=(2,2))
				cropped_frame = clahe.apply(cropped_frame) 

				# Resize image and extract facial landmark features
				cropped_frame = cv2.resize(cropped_frame, (IMAGE_SIZE,IMAGE_SIZE))
				features = self.getFL(cropped_frame)
				features = (features-mean_fl)/std_fl

				# Do necessary preprocessing
				cropped_frame = cropped_frame.reshape((1,IMAGE_SIZE,IMAGE_SIZE,NUM_CHANNELS))
				cropped_frame = (cropped_frame-np.mean(cropped_frame))/np.std(cropped_frame)

				# Feed the cropped and preprocessed frame to classifier
				result = self.sess.run(self.output, {self.x_input:cropped_frame, self.fl:features})

				# Get the emotion
				emotion = EMOTIONS[np.argmax(result)]

				# Store emotion data
				emotionData = SEmotion()
				emotionData.x = x1
				emotionData.y = y1
				emotionData.w = abs(x2-x1)
				emotionData.h = abs(y2-y1)
				emotionData.emotion = emotion
				emotions_temp.append(emotionData)

				# For testing purpose
				cv2.imshow("Image Fed to Classifier", cropped_frame.reshape((IMAGE_SIZE, IMAGE_SIZE)))

			self.emotionList = emotions_temp

		except Ice.Exception, e:
			traceback.print_exc()
			print e

		cv2.waitKey(1)
		return True

	def getEmotionList(self):
		return self.emotionList