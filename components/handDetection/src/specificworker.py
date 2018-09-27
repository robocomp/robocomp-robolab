#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2018 by Esteban Martinena
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
from libs.HandDetection.HandDetection import HandDetector
from scipy import stats
import numpy as np
import cv2


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 20
		self.hand_detector = HandDetector(-1)
		self.hand_detector.set_mask_mode("depth")
		self.timer.start(self.Period)
		self.state = "None"
		self.new_hand_roi = None
		self.expected_hands = 0
		self.hand_detector.debug = False
		self.depth_thresold = 0
		self.flip = False
		self.debug = False
		# It helps to increase the space between the depth wall and some inclination or frames



	def setParams(self, params):
		#try:
		#	self.innermodel = InnerModel(params["InnerModelPath"])
		#except:
		#	traceback.print_exc()
		#	print "Error reading config params"
		if "debug" in params:
			if "true" in params["debug"].lower():
				self.hand_detector.debug=True
				self.debug = True
				search_roi_class = TRoi()
				search_roi_class.y = 480 / 2 - 100
				search_roi_class.x = 640 / 2 - 100
				search_roi_class.w = 200
				search_roi_class.h = 200
				search_roi = (
					search_roi_class.x, search_roi_class.y, search_roi_class.h, search_roi_class.w)
				self.addNewHand(1, search_roi_class)
		if "flip" in params:
			if "true" in params["flip"].lower():
				self.flip = True
		return True

	@QtCore.Slot()
	def compute(self):
		try:
			# image = self.camerasimple_proxy.getImage()
			# frame = np.fromstring(image.image, dtype=np.uint8)
			# frame = frame.reshape(image.width, image.height, image.depth)

			color, depth, _, _ = self.rgbd_proxy.getData()
			frame = np.fromstring(color, dtype=np.uint8)
			frame = frame.reshape(480, 640, 3)

			depth = np.array(depth, dtype=np.float32)
			if self.depth_thresold < 1:
				self.calculate_depth_threshold(depth)


			# depth = np.array(depth, dtype=np.uint8)
			# depth = depth.reshape(480, 640, 1)
			if self.flip:
				frame = cv2.flip(frame,0)
				depth = np.fromstring(depth, dtype=np.float32)
				depth = depth.reshape(480, 640, 1)
				depth = cv2.flip(depth,0)
				depth = depth.reshape(480*640)

			print "showing depth"
			self.hand_detector.set_depth_mask(np.array(depth))
			if self.debug:
				depth_to_show = depth.reshape(480, 640, 1)
				cv2.imshow("DEBUG: HandDetection: Specificworker: depth readed ", depth_to_show)
				cv2.imshow("DEBUG: HandDetection: Specificworker: color", frame)


		except Ice.Exception, e:
			traceback.print_exc()
			print e
			return False

		if self.state == "add_new_hand":
			search_roi = (self.new_hand_roi.x, self.new_hand_roi.y, self.new_hand_roi.w, self.new_hand_roi.h)
			self.hand_detector.add_hand2(frame, search_roi)
			if len(self.hand_detector.hands) >= self.expected_hands:
				self.state = "tracking"
				self.new_hand_roi = None
		elif self.state == "tracking":
			self.hand_detector.update_detection_and_tracking(frame)
		print "Compute in state %s with %d hands" % (self.state, len(self.hand_detector.hands))
		return True

	def depth_normalization(self, depth):
		depth_min = np.min(depth)
		depth_max = np.max(depth)
		if depth_max != depth_min and depth_max > 0:
			depth = np.interp(depth, [depth_min, depth_max], [0.0, 255.0], right=255, left=0)
		return depth

	def calculate_depth_threshold(self, depth):
		depth = np.array(depth)
		criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
		# Create 3 clusters of point (0, intermediate, far)
		ret, label, center = cv2.kmeans(depth.astype(np.float32), 3, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
		clusters = [depth[label.ravel() == 0], depth[label.ravel() == 1], depth[label.ravel() == 2]]
		# The wall or table probably is the bigger one of the clusters
		index = np.argmax(map(len, clusters))
		# print np.median(clusters[index])
		# print np.mean(clusters[index])
		# Use the most repeated value as the depth to wich the wall or table can be.
		# TODO: ENV_DEPENDECE: 70 is the amount of space for the frame of the tv or for inclination from the camera plane
		self.depth_thresold = stats.mode(clusters[index])[0][0]-70
		self.hand_detector.set_depth_threshold(self.depth_thresold)


	def addNewHand(self, expected_hands, new_hand_roi):
		self.new_hand_roi = new_hand_roi
		self.expected_hands = expected_hands
		self.state = "add_new_hand"
		return self.expected_hands


	def getHands(self):
		ret = []
		for detected_hand in self.hand_detector.hands:
			new_hand = Hand()
			new_hand.id = detected_hand.id
			new_hand.score = detected_hand.truth_value
			new_hand.fingertips = detected_hand.fingertips
			new_hand.intertips = detected_hand.intertips
			new_hand.positions = detected_hand.position_history
			# print detected_hand.contour
			new_hand.contour = detected_hand.contour
			new_hand.centerMass = detected_hand.center_of_mass
			new_hand.truthValue = detected_hand.truth_value
			new_hand.detected = detected_hand.detected
			new_hand.tracked = detected_hand.tracked
			ret.append(new_hand)
		return ret


	def getHandsCount(self):
		return len(self.hand_detector.hands)





