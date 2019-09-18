#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2019 by Robocomp
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
from datetime import datetime

from libs.HandDetection.Hand import Hand
from libs.HandDetection.roi import Roi, SIDE
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
		self.Period = 30
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
		self.fps = 0
		self.last_time = 0
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

			color, depth, _, _ = self.rgbd_proxy.getData()

			frame = np.fromstring(color, dtype=np.uint8)
			frame = frame.reshape(480, 640, 3)

			depth = np.array(depth, dtype=np.float32)
			depth = np.fromstring(depth, dtype=np.float32)
			if self.depth_thresold < 1:
				self.calculate_depth_threshold(depth)


			# depth = np.array(depth, dtype=np.uint8)
			# depth = depth.reshape(480, 640, 1)
			if self.flip:
				frame = cv2.flip(frame,0)
				depth = depth.reshape(480, 640, 1)
				depth = cv2.flip(depth,0)
				depth = depth.reshape(480*640)

			self.hand_detector.set_depth_mask(np.array(depth))
			if self.debug:
				depth_to_show = self.depth_normalization(depth).reshape(480, 640, 1).astype(np.uint8)
				frame_to_show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
				cv2.imshow("DEBUG: HandDetection: Specificworker: depth readed ", depth_to_show)
				cv2.imshow("DEBUG: HandDetection: Specificworker: color", frame_to_show)


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
			for detected_hand in self.hand_detector.hands:
				try:
					points, _, _ = self.rgbd_proxy.getXYZByteStream()
					np_points = np.fromstring(points, dtype=np.float32)
					points_frame = np_points.reshape(480, 640, 3)
					if self.flip:
						points_frame = cv2.flip(points_frame, 0)
					# print len(np_points)ç
					cm = detected_hand.center_of_mass
					# print ("%s %s"%(str(detected_hand.center_of_mass), str(points_frame.shape)))
					point_x = points_frame[cm[1], cm[0], 0]
					point_y = points_frame[cm[1], cm[0], 1]
					point_z = points_frame[cm[1], cm[0], 2]
					# IT'S A 640x480 matrix
					# point_x = points_frame[360, 240, 0]
					# point_y = points_frame[360, 240, 1]
					# point_z = points_frame[360, 240, 2]
					detected_hand.center_of_mass_xyz = [int(point_x), int(point_y), int(point_z)]
					if self.debug:
						print("%d %d %d" % (int(detected_hand.center_of_mass_xyz[0]), int(detected_hand.center_of_mass_xyz[1]), int(detected_hand.center_of_mass_xyz[2])))
						# print("%d %d " % (cm[0], cm[1]))
						xyz_to_show = self.depth_normalization(points_frame[:,:,2]).astype(np.uint8)
						# print xyz_to_show.shape
						# gray_xyz = cv2.cvtColor(xyz_to_show, cv2.COLOR_BGR2GRAY)
						cv2.circle(xyz_to_show, (cm[0],cm[1]), 20, (0, 0, 255), -1)
						font = cv2.FONT_HERSHEY_SIMPLEX
						cv2.putText(xyz_to_show, str(cm), cm, font, 1, (255, 255, 255), 2, cv2.LINE_AA)
						cv2.imshow("XYZ", xyz_to_show)
						# print("%d" % (len(detected_hand.center_of_mass)))
						# print("%f %f %f" % (int(point_x), int(point_y), int(point_z)))

				except Exception as e:
					print("Problem getting points (XYZ) from rgbd")
					print e
		self.calculate_fps(False)
		# print "%s with %d of %d hands (Mode: %s, FPS: %d)" % (self.state, len(self.hand_detector.hands), self.expected_hands, self.hand_detector.mask_mode, self.fps)
		return True

	def calculate_fps(self, to_print=False):
		current_time = time.time()
		self.fps=1.0 / (current_time - self.last_time)
		self.last_time = current_time
		if to_print:
			print("%d fps"%(self.fps))


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
			new_hand.centerMass2D = detected_hand.center_of_mass
			try:
				new_hand.centerMass3D = detected_hand.center_of_mass_xyz
				print("%d %d %d" % (int(new_hand.centerMass3D[0]), int(new_hand.centerMass3D[1]), int(new_hand.centerMass3D[2])))
			except Exception as e:
				new_hand.centerMass3D = [0,0,0]
				print "no xyz center of mass"
				print e
			# print("%d %d"%(len(new_hand.centerMass),len(detected_hand.center_of_mass)))
			new_hand.truthValue = detected_hand.truth_value
			new_hand.detected = detected_hand.detected
			new_hand.tracked = detected_hand.tracked

			ret.append(new_hand)
		return ret


	def getHandsCount(self):
		return len(self.hand_detector.hands)





