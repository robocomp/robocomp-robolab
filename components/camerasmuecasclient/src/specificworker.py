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
		self.Period = 70
		self.timer.start(self.Period)
		self.innermodel
		
	def setParams(self, params):
		try:
			self.innermodel = InnerModel(params["/home/robocomp/robocomp/files/innermodel/muecas-robolab.xml"])
		except:
			traceback.print_exc()
			print "Error reading config params"
		return True

	@QtCore.Slot()
	def compute(self):
		#print 'SpecificWorker.compute...'
		try:
			data = self.camerasmuecas_proxy.getImages()
			arrL = np.fromstring(data.leftImg, np.uint8)
			arrR = np.fromstring(data.rightImg, np.uint8)
			self.imgL = np.reshape(arrL,(data.width, data.height, data.depth))
			self.imgR = np.reshape(arrR,(data.width, data.height, data.depth))
			self.drawHair()
			cv2.imshow('frameL',self.imgL)
			cv2.imshow('frameR',self.imgR)
		except Ice.Exception, e:
			traceback.print_exc()
			print e
			
		cv2.waitKey(1)
		return True
		
	#################################
	# Draw a cross in the middle
	#
	def drawHair(self):
		rows,cols,depth =  self.imgL.shape
		cv2.line(self.imgL,(cols/2,rows/2-50),(cols/2,rows/2+50),(0,255,0),2)
		cv2.line(self.imgL,(cols/2-50,rows/2),(cols/2+50,rows/2),(0,255,0),2)
		cv2.line(self.imgR,(cols/2,rows/2-50),(cols/2,rows/2+50),(0,0,255),2)
		cv2.line(self.imgR,(cols/2-50,rows/2),(cols/2+50,rows/2),(0,0,255),2)
