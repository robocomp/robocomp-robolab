#
# Copyright (C) 2015 by YOUR NAME HERE
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

import sys, os, Ice, traceback

from PySide import *
from genericworker import *
import cv2

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	pass
if len(ROBOCOMP)<1:
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"RGBDBus.ice")
from RoboCompRGBDBus import *

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 1000
		self.timer.start(self.Period)

	def setParams(self, params):
		#try:
		#	par = params["InnerModelPath"]
		#	innermodel_path=par.value
		#	innermodel = InnerModel(innermodel_path)
		#except:
		#	tracebak.print_exc()
		#	print "Error reading config params"
		cv2.namedWindow("img")
		return True

	@QtCore.Slot()
	def compute(self):
		#print 'SpecificWorker.compute...'
		#try:
			#images = RoboCompRGBDBus.ImageMap()
			#self.rgbdbus_proxy.getImages( ["default"], images)
			#print images.keys(), len(images)
			#if "default" in images.keys():
				#cv2.imshow("img", images["default"])
			#else:
				#print "Camera not found in server"
		#except Ice.Exception, e:
			#traceback.print_exc()
			#print e
			
		
		
		return True





