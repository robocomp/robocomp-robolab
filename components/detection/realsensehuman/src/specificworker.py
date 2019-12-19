#!/usr/bin/python3
# -*- coding: utf-8 -*-
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

from genericworker import *
import pyrealsense2 as rs
import torch
import numpy as np
import cv2
from openpifpaf.network import nets
from openpifpaf import decoder, show, transforms
import argparse
import time
# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

COCO_IDS=["nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder", "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist", "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle" ]
SKELETON_CONNECTIONS = [("left_ankle", "left_knee"),
						("left_knee", "left_hip"),
						("right_ankle", "right_knee"),
						("right_knee", "right_hip"),
						("left_hip", "right_hip"),
						("left_shoulder", "left_hip"),
						("right_shoulder", "right_hip"),
						("left_shoulder", "right_shoulder"),
						("left_shoulder", "left_elbow"),
						("right_shoulder", "right_elbow"),
						("left_elbow", "left_wrist"),
						("right_elbow", "right_wrist"),
						("left_eye", "right_eye"),
						("nose", "left_eye"),
						("nose", "right_eye"),
						("left_eye", "left_ear"),
						("right_eye", "right_ear"),
						("left_ear", "left_shoulder"),
						("right_ear", "right_shoulder")]


class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 20
		self.timer.start(self.Period)
		self.initialize()

	def __del__(self):
		print('SpecificWorker destructor')
		self.pipeline.stop()

	def setParams(self, params):
		#try:
		#	self.innermodel = InnerModel(params["InnerModelPath"])
		#except:
		#	traceback.print_exc()
		#	print("Error reading config params")

		return True

	def initialize(self):
		#openpifpaf configuration
		class Args:
			source = 0
			checkpoint = None
			basenet = None
			dilation = None
			dilation_end = None
			headnets = ['pif', 'paf']
			dropout = 0.0
			quad = 1
			pretrained = False
			keypoint_threshold = None
			seed_threshold = 0.2
			force_complete_pose = False
			debug_pif_indices = []
			debug_paf_indices = []
			connection_method = 'max'
			fixed_b = None
			pif_fixed_scale = None
			profile_decoder = False
			instance_threshold = 0.05
			device = torch.device(type="cpu")
			disable_cuda = False
			scale = 1
			key_point_threshold = 0.05
			head_dropout = 0.0
			head_quad = 0
			default_kernel_size = 1
			default_padding = 0
			default_dilation = 1
			head_kernel_size = 1
			head_padding = 0
			head_dilation = 0

		self.args = Args()
		model, _ = nets.factory_from_args(self.args)
		model = model.to(self.args.device)
		# model.cuda()
		self.processor = decoder.factory_from_args(self.args, model)

		#realsense configuration
		try:

			config = rs.config()
			config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
			config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
			self.pipeline = rs.pipeline()
			self.pipeline.start(config)
		except Exception as e:
			print(e)
			exit(-1)
		#data
		self.src = np.zeros((480, 640, 3), np.uint8)




	@QtCore.Slot()
	def compute(self):
		start = time.time()
		print('SpecificWorker.compute...')
		frames = self.pipeline.wait_for_frames()
		self.depth = frames.get_depth_frame()
		self.color = np.asanyarray(frames.get_color_frame().get_data())
		if not self.depth:
			return

		self.processImage(0.3)
		cv2.imshow("Color frame", self.color)
		cv2.waitKey(1)
		
		print("FPS:", 1/(time.time()-start))
		return True

	def processImage(self, scale):
		image = cv2.resize(self.color, None, fx=scale, fy=scale)
		processed_image_cpu = transforms.image_transform(image)
		processed_image = processed_image_cpu.contiguous().to(non_blocking=True)
		unsqueezed = torch.unsqueeze(processed_image, 0).to(self.args.device)
		fields = self.processor.fields(unsqueezed)[0]
		keypoint_sets, _ = self.processor.keypoint_sets(fields)

		# create joint dictionary
		joints = dict()
		for pos, joint in enumerate(keypoint_sets[0]):
				joints[COCO_IDS[pos]] = [joint[0] / scale, joint[1] / scale, joint[2]]

		#draw
		for name1, name2 in SKELETON_CONNECTIONS:
			joint1 = joints[name1]
			joint2 = joints[name2]
			if joint1[2] > 0.5:
				cv2.circle(self.color, (int(joint1[0]), int(joint1[1])), 10, (0, 0, 255))
			if joint2[2] > 0.5:
				cv2.circle(self.color, (int(joint2[0]), int(joint2[1])), 10, (0, 0, 255))
			if joint1[2] > 0.5 and joint2[2] > 0.5:
				cv2.line(self.color, (int(joint1[0]), int(joint1[1])), (int(joint2[0]), int(joint2[1])), (0, 255, 0), 2)


	# =============== Methods for Component Implements ==================
# ===================================================================

	#
	# getData
	#
	def getData(self):
		#
		# implementCODE
		#
		rgbMatrix = []
		distanceMatrix = []
		for y in range(480):
			for x in range(640):
				distanceMatrix.append(self.depth.get_distance(x, y))

		hState = {}
		bState = RoboCompGenericBase.TBaseState()
		return [rgbMatrix, distanceMatrix, hState, bState]


	#
	# getDepth
	#
	def getDepth(self):
		#
		# implementCODE
		#
		depth = []
		for y in range(480):
			for x in range(640):
				depth.append(self.depth.get_distance(x, y))
		hState = {}
		bState = TBaseState()
		return (depth, hState, bState)


	#
	# getDepthInIR
	#
	def getDepthInIR(self):
		#
		# implementCODE
		#
		distanceMatrix = []
		hState = {}
		bState = RoboCompGenericBase.TBaseState()
		return [distanceMatrix, hState, bState]


	#
	# getImage
	#
	def getImage(self):
		#
		# implementCODE
		#
		color = []
		depth = []
		points = []
		hState = []
		bState = RoboCompGenericBase.TBaseState()
		return [color, depth, points, hState, bState]


	#
	# getRGB
	#
	def getRGB(self):
		#
		# implementCODE
		#
		color = []
		hState = {}
		bState = RoboCompGenericBase.TBaseState()
		return [color, hState, bState]


	#
	# getRGBDParams
	#
	def getRGBDParams(self):
		ret = TRGBDParams()
		#
		# implementCODE
		#
		return ret


	#
	# getRegistration
	#
	def getRegistration(self):
		ret = Registration()
		#
		# implementCODE
		#
		return ret


	#
	# getXYZ
	#
	def getXYZ(self):
		#
		# implementCODE
		#
		points = []
		hState = {}
		bState = RoboCompGenericBase.TBaseState()
		return [points, hState, bState]


	#
	# getXYZByteStream
	#
	def getXYZByteStream(self):
		#
		# implementCODE
		#
		pointStream = []
		hState = {}
		bState = TBaseState()
		return [pointStream, hState, bState]


	#
	# setRegistration
	#
	def setRegistration(self, value):
		#
		# implementCODE
		#
		pass

# ===================================================================
# ===================================================================

