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

from PySide import QtGui, QtCore
from genericworker import *
import cv2
import numpy as np
from PyQt4.QtGui import QImage
from PyQt4.QtCore import QSize

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 5
		self.timer.start(self.Period)

	def setParams(self, params):
		#try:
		#	self.innermodel = InnerModel(params["InnerModelPath"])
		#except:
		#	traceback.print_exc()
		#	print "Error reading config params"
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'
		#computeCODE
		try:
			color, depth, _, _= self.rgbd_proxy.getData()
			# print depth
			image = np.frombuffer(color, dtype=np.uint8)
			print image.size
			image =np.reshape(image, (480,640,3))
			image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
			cv2.imshow("color", image)
			print (min(depth), max(depth))
			depth = np.interp(depth, [min(depth), max(depth)], [0, 255])
			print(min(depth), max(depth))
			depth = np.reshape(depth, (480,640,1))
			cv2.imshow("depth", depth.astype(np.uint8))
			k = cv2.waitKey(5)
			#print len(color)
		except Ice.Exception, e:
			traceback.print_exc()
			print e


		return True

