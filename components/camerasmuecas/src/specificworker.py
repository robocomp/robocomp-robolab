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
sys.path.append('/opt/robocomp/lib')
#import librobocomp_qmat
#import librobocomp_osgviewer
#import librobocomp_innermodel

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 10
		self.timer.start(self.Period)
		self.frames = 0
		self.correct = True
		self.start = time.time()
		#self.innermodel

	def setParams(self, params):
		self.capL = cv2.VideoCapture(0)
		self.capR = cv2.VideoCapture(1)
		#try:
			#self.innermodel = InnerModel(params["/home/robocomp/robocomp/files/innermodel/muecas-robolab.xml"])
		#except:
			#traceback.print_exc()
			#print "Error reading config params"
		#return True

	@QtCore.Slot()
	def compute(self):
		retL, frameL = self.capL.read()
		retR, frameR = self.capR.read()
		
		rows,cols,depth =  frameL.shape
		if rows == 0 or cols == 0 or depth == 0:
			self.correct = False
			return False
		else:
			self.correct = True
			
		M = cv2.getRotationMatrix2D((cols/2,rows/2),180,1)
		self.imgL = cv2.warpAffine(frameL, M,(cols,rows))
		self.imgR = cv2.warpAffine(frameR, M,(cols,rows))
		
		self.printFrames()
		return True

	# Print fps 
	#
	def printFrames(self):
		end = time.time()
		if (end - self.start) < 1:
			self.frames += 1
			return 
		print(str(int(self.frames/ (end-self.start))) + " fps")
		self.start = end
		self.frames = 0
	
	########################################
	# Middleware interface
	# GetImages from remote client
	########################################
	def getImages(self):
		if self.correct == True:
			ret = ImagePair()
			ret.leftImg = self.imgL.data
			ret.rightImg = self.imgR.data
			ret.width, ret.height, ret.depth = self.imgL.shape
			return ret
		else:
			print "Error reading images..."
			raise RoboCompCamerasMuecas.HardwareFailedException()
			
			
			

