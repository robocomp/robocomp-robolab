#
# Copyright (C) 2017 by YOUR NAME HERE
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


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 50
		self.timer.start(self.Period)

	def setParams(self, params):
		#try:
		#	self.innermodel = InnerModel(params["InnerModelPath"])
		#except:
		#	traceback.print_exc()
		#	print "Error reading config params"
		
		self.capL = cv2.VideoCapture(0)
		#self.capR = cv2.VideoCapture(1)
		return True
    
	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'

		# The API of python-innermodel is not exactly the same as the C++ version
		# self.innermodel.updateTransformValues("head_rot_tilt_pose", 0, 0, 0, 1.3, 0, 0)
		# z = librobocomp_qmat.QVec(3,0)
		# r = self.innermodel.transform("rgbd", z, "laser")
		# r.printvector("d")
		# print r[0], r[1], r[2]
		retL, frameL = self.capL.read()
		#retR, frameR = self.capR.read()
		# Our operations on the frame come here
		# grayL = cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY)
		# grayR = cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY)
		
		rows,cols,depth =  frameL.shape
		M = cv2.getRotationMatrix2D((cols/2,rows/2),180,1)
		self.imgL = cv2.warpAffine(frameL, M,(cols,rows))
		#self.imgR = cv2.warpAffine(frameR, M,(cols,rows))
		
		# Display the resulting frame
		#cv2.imshow('frameL',self.imgL)
		#cv2.imshow('frameR',self.imgR)
		#cv2.waitKey(1)
		return True
    
	#
	# SERVANTS ---------------------  getImage
	#
	def getImage(self):
		#
		#implementCODE
		#
		im = TImage()
		im.image = self.imgL.data
		im.width, im.height, im.depth = self.imgL.shape
		return im

