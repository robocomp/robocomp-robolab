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

import imutils

from genericworker import *
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
		self.timer.start(self.Period)

		self.debug = False
		self.fps = 0
		self.greenLower = (29, 86, 6)
		self.greenUpper = (64, 255, 255)
		cv2.namedWindow("image")
		cv2.setMouseCallback("image", self.click_event)
		self.refPt =[]
		self.cropping = False
		self.lower_hsv = None
		self.upper_hsv = None
		self.state = "initial"

		# It helps to increase the space between the depth wall and some inclination or frames

	def click_event(self, event, x, y, flags, param):

		# if the left mouse button was clicked, record the starting
		# (x, y) coordinates and indicate that cropping is being
		# performed
		if event == cv2.EVENT_LBUTTONDOWN:
			self.refPt = [(x, y)]
			self.cropping = True

		# check to see if the left mouse button was released
		elif event == cv2.EVENT_LBUTTONUP:
			# record the ending (x, y) coordinates and indicate that
			# the self.cropping operation is finished
			self.refPt.append((x, y))
			self.cropping = False



	def setParams(self, params):
		#try:
		#	self.innermodel = InnerModel(params["InnerModelPath"])
		#except:
		#	traceback.print_exc()
		#	print "Error reading config params"
		if "debug" in params:
			if "true" in params["debug"].lower():
				self.debug = True

		if "flip" in params:
			if "true" in params["flip"].lower():
				self.flip = True
		return True

	@QtCore.Slot()
	def compute(self):
		self.last_time = time.time()
		try:
			# image = self.camerasimple_proxy.getImage()
			# frame = np.fromstring(image.image, dtype=np.uint8)
			# frame = frame.reshape(image.width, image.height, image.depth)

			color, depth, _, _ = self.rgbd_proxy.getData()
			frame = np.fromstring(color, dtype=np.uint8)
			frame = frame.reshape(480, 640, 3)

			depth = np.array(depth, dtype=np.float32)


			# depth = np.array(depth, dtype=np.uint8)
			# depth = depth.reshape(480, 640, 1)
			if self.flip:
				frame = cv2.flip(frame,0)
				depth = np.fromstring(depth, dtype=np.float32)
				depth = depth.reshape(480, 640, 1)
				depth = cv2.flip(depth,0)
				depth = depth.reshape(480*640)

			frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
			white_balanced = white_balance(frame)
			if "initial" in self.state:
				print "Seetting reference colors"
				# label_image = self.label_image(frame)
				floodflags = 4
				floodflags |= cv2.FLOODFILL_MASK_ONLY
				floodflags |= (255 << 8)
				filled = white_balanced.copy()
				h, w, chn = frame.shape
				mask1 = np.zeros((h + 2, w + 2), np.uint8)
				mask2 = np.zeros((h + 2, w + 2), np.uint8)
				image_mask_image = np.zeros((h , w), np.uint8)
				if len(self.refPt)>0:
					cv2.floodFill(filled, mask1, self.refPt[0], (0,0,255), (10, 10, 10), (10, 10, 10), floodflags)
					hsv_frame = cv2.cvtColor(white_balanced, cv2.COLOR_RGB2HSV)
					img_mask = hsv_frame[np.where(mask1 == 255)]
					# img_avg = np.mean(img_mask, axis=0)
					min_hsv = np.min(img_mask, axis=0)
					min_hsv[2] = 0
					max_hsv = np.max(img_mask, axis=0)
					max_hsv[2] = 255
					self.lower_hsv = min_hsv
					self.upper_hsv = max_hsv
				if len(self.refPt) > 1:
					cv2.floodFill(filled, mask2, self.refPt[1], (0, 0, 255), (10, 10, 10), (10, 10, 10), floodflags)
					self.state = "tracking"
			if "tracking" in self.state:
				print "Tracking"
				color_tracked_frame, mask = self.get_color_mask(white_balanced)
				cv2.imshow("DEBUG: HandDetection: Specificworker: color_filter", color_tracked_frame)





			if self.debug:
				depth_to_show = self.depth_normalization(depth).reshape(480, 640, 1).astype(np.uint8)
				frame_to_show_wb = cv2.cvtColor(white_balanced, cv2.COLOR_BGR2RGB)
				frame_to_show = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
				# cv2.imshow("DEBUG: HandDetection: Specificworker: depth readed ", depth_to_show)
				# cv2.imshow("DEBUG: HandDetection: Specificworker: color", frame_to_show)
				# cv2.imshow("DEBUG: HandDetection: Specificworker: color_wb", frame_to_show_wb)

				# cv2.imshow("DEBUG: HandDetection: Specificworker: color_filter", mask)
				# cv2.imshow("DEBUG: HandDetection: Specificworker: mask", image_mask_image)
				# cv2.imshow("DEBUG: HandDetection: Specificworker: mask", image_mask_image)
				# cv2.imshow("DEBUG: HandDetection: Specificworker: filled", filled)
				cv2.imshow("image", frame)
				# cv2.imshow("mask", mask1)



		except Ice.Exception, e:
			traceback.print_exc()
			print e
			return False

		return True

	def calculate_fps(self):
		self.fps=1.0 / (time.time() - self.last_time)


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

	def label_image(self, frame):
		new_frame = frame.copy()
		new_frame = cv2.threshold(new_frame, 127, 255, cv2.THRESH_BINARY)[1]  # ensure binary
		ret, labels = cv2.connectedComponents(new_frame)

		# Map component labels to hue val
		label_hue = np.uint8(179 * labels / np.max(labels))
		blank_ch = 255 * np.ones_like(label_hue)
		labeled_img = cv2.merge([label_hue, blank_ch, blank_ch])

		# cvt to BGR for display
		labeled_img = cv2.cvtColor(labeled_img, cv2.COLOR_HSV2BGR)

		# set bg label to black
		labeled_img[label_hue == 0] = 0

		return labeled_img

	def get_color_mask(self, frame):
		new_frame = frame.copy()
		mask =  np.zeros((480, 640), np.uint8)
		if self.lower_hsv is None or self.upper_hsv is None:
			return new_frame, mask
		new_frame = frame.copy()
		blurred = cv2.GaussianBlur(new_frame, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

		# construct a mask for the color "green", then perform
		# a series of dilations and erosions to remove any small
		# blobs left in the mask
		mask = cv2.inRange(hsv, self.greenLower, self.greenUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)
		# find contours in the mask and initialize the current
		# (x, y) center of the ball
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
								cv2.CHAIN_APPROX_SIMPLE)
		cnts = cnts[0] if imutils.is_cv2() else cnts[1]
		center = None
		if len(cnts) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

			# only proceed if the radius meets a minimum size
			if radius > 10:
				# draw the circle and centroid on the new_frame,
				# then update the list of tracked points
				cv2.circle(new_frame, (int(x), int(y)), int(radius),
						   (0, 255, 255), 2)
				cv2.circle(new_frame, center, 5, (0, 0, 255), -1)
		return new_frame, mask




def white_balance(img):
    result = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    avg_a = np.average(result[:, :, 1])
    avg_b = np.average(result[:, :, 2])
    result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result = cv2.cvtColor(result, cv2.COLOR_LAB2BGR)
    return result