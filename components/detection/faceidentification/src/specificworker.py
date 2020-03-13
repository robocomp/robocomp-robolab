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

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import sys, os, traceback, time
sys.path.append(os.path.join(os.getcwd(),"assets","src"))
print(sys.path)

from PySide import QtGui, QtCore
from genericworker import *
import numpy as np
import math
import cv2
from scipy.optimize import brentq
from scipy import interpolate
from sklearn import metrics
import facenet2 as facenet
#from assets.src import lfw
import scipy.io
from scipy import misc
import operator
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 
import tensorflow as tf


## Normalizing the image for finding feature vector
def to_rgb(img):
	w, h = img.shape
	ret = np.empty((w, h, 3), dtype=np.uint8)
	ret[:, :, 0] = ret[:, :, 1] = ret[:, :, 2] = img
	return ret

def prewhiten(x):
	mean = np.mean(x)
	std = np.std(x)
	std_adj = np.maximum(std, 1.0/np.sqrt(x.size))
	y = np.multiply(np.subtract(x, mean), 1/std_adj)
	return y 

def preprocess_image(img, image_size, do_prewhiten=True):
	images = np.zeros((1, image_size, image_size, 3))
	if img.ndim == 2:
			img = to_rgb(img)
	if do_prewhiten:
			img = prewhiten(img)
	img = cv2.resize(img, (160,160),interpolation=cv2.INTER_CUBIC)
	images[:,:,:,:] = img
	return images


## Calculating the distance between given 2 feature vectors.
def cal_face_dist(train,test):
	train = train[0]
	if ((train == test).all()):
		return 1e6
	else:
		return 1/(np.linalg.norm(train-test))							### Thresh_1 = 1.10
		# return np.sum(train*test)										### Thresh_2 = 0.55


class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)

		### Checking if embedding file exists or not
		files = os.listdir('.')
		self.file_name = 'data_embeddings.npy'
		if (self.file_name in files):
			print ('Embeddings file found')
			self.neural_embeddings = np.load(self.file_name, allow_pickle=True)
		else:
			### Creating an embedding file if no embeddings exist already
			print ('No File Found. Creating an empty numpy file for neural embeddings')
			self.neural_embeddings = np.zeros((1,2), dtype = np.ndarray)
			self.neural_embeddings[0,0] = np.zeros((1,512), dtype = np.float32)			
			self.neural_embeddings[0,1] = '####'
			np.save(self.file_name, self.neural_embeddings)

		self.timer.timeout.connect(self.compute)
		self.Period = 50
		self.timer.start(self.Period)

		##### Defining the face recognition model and parameters
		self.model_path = './assets/20180408-102900/'
		##### Thresholds for matching the neural embeddings for label prediction
		self.threshold_1 = 1.10
		self.threshold_2 = 0.55
		##### Relaxed threshold to check if incorrect label is given
		self.threshold_3 = 1.25
		##### Threshold to remove redundant data
		self.threshold_4 = 3
		self.image_size = 160

		#### Loading the Face Recognition Model
		with tf.Graph().as_default():  
			self.config = tf.ConfigProto()
			self.config.gpu_options.per_process_gpu_memory_fraction = 0.3
			self.sess = tf.InteractiveSession(config=self.config)
			facenet.load_model(self.model_path)
			self.images_placeholder = tf.get_default_graph().get_tensor_by_name("input:0")
			self.images_placeholder = tf.image.resize_images(self.images_placeholder,(self.image_size,self.image_size))
			self.embeddings = tf.get_default_graph().get_tensor_by_name("embeddings:0")
			self.phase_train_placeholder = tf.get_default_graph().get_tensor_by_name("phase_train:0")

	def setParams(self, params):
		return True

	def compare_embeddings(self, test_face_embedding, thresh):
		possible_faces_score = {}
		possible_faces_weight = {}
		possible_faces_count = {}
		for idx in range(self.neural_embeddings.shape[0]):
			dist = cal_face_dist(self.neural_embeddings[idx,0], test_face_embedding[0])
			if (dist > thresh):
				if (self.neural_embeddings[idx,1] not in possible_faces_score):
					possible_faces_count[self.neural_embeddings[idx,1]] = 1
					possible_faces_score[self.neural_embeddings[idx,1]] = dist
					possible_faces_weight[self.neural_embeddings[idx,1]] = 1
				else:
					val = possible_faces_score[self.neural_embeddings[idx,1]] * possible_faces_count[self.neural_embeddings[idx,1]]
					possible_faces_weight[self.neural_embeddings[idx,1]] = possible_faces_weight[self.neural_embeddings[idx,1]] + 0.5
					possible_faces_count[self.neural_embeddings[idx,1]] = possible_faces_count[self.neural_embeddings[idx,1]] + 1
					possible_faces_score[self.neural_embeddings[idx,1]] = (val + (dist*possible_faces_weight[self.neural_embeddings[idx,1]])) / possible_faces_count[self.neural_embeddings[idx,1]]
		if not possible_faces_score:
			return "Unknown", -1
		pred_label = max(possible_faces_score.iteritems(), key=operator.itemgetter(1))[0]
		if (pred_label == '####'):
			return "Unknown", -1
		return pred_label, possible_faces_score[pred_label]

	@QtCore.Slot()
	def compute(self):
		return True

	#### Deleting an already existing label
	def deleteLabel(self, faceLabel):
		flag = 0
		count = 0
		for idx, label in enumerate(self.neural_embeddings):
			if (label[1] == faceLabel):
				self.neural_embeddings = np.delete(self.neural_embeddings, idx - count, axis = 0)
				count = count + 1
				flag = 1
			else:
				pass
		if (flag == 0):
			print ('No person found with the given name')
		else:
			np.save(self.file_name, self.neural_embeddings)


	#### Returning a facelabel for the given bounding box face
	def getFaceLabels(self, faceImg):
		arr = np.fromstring(faceImg.image, np.uint8)
		image = np.reshape(arr, (faceImg.width, faceImg.height, faceImg.depth))
		image = preprocess_image(image, self.image_size)
		feed_dict = {self.images_placeholder:image, self.phase_train_placeholder:False}
		feature_vector = self.sess.run(self.embeddings, feed_dict=feed_dict)		
		prediction, number_matches = self.compare_embeddings(feature_vector, self.threshold_1)
		faceLabel = prediction
		return faceLabel

	#### Adding new faces to the database
	def addNewFace(self, faceImg, faceLabel):
		arr = np.fromstring(faceImg.image, np.uint8)
		image = np.reshape(arr, (faceImg.width, faceImg.height, faceImg.depth))
		image = preprocess_image(image, self.image_size)
		feed_dict = {self.images_placeholder:image, self.phase_train_placeholder:False }
		feature_vector = self.sess.run(self.embeddings, feed_dict=feed_dict)
		
		#### Data Reduction and Error check on the Label
		prediction, number_matches = self.compare_embeddings(feature_vector, self.threshold_4)
		#### If new image is very similar to the old image, then it is not added to the database.
		if (prediction == faceLabel):
			return True

		#### Check whether given image is not similar to a different label in the database.
		prediction, number_matches = self.compare_embeddings(feature_vector, self.threshold_3)
		if (prediction == "Unknown" or prediction == faceLabel):
			temp = np.array([[feature_vector, faceLabel]])
			self.neural_embeddings = np.concatenate((self.neural_embeddings, temp),axis = 0)
			np.save(self.file_name, self.neural_embeddings)
			return True
		else:
			return False
