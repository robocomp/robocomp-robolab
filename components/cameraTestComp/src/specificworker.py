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
from cv2 import cv
import numpy as np

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
		self.Period = 100
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
		print 'SpecificWorker.compute...'
		try:
			images = self.rgbdbus_proxy.getImages( ["default"])
			if len(images) > 0:
				image = images["default"]
				#nparr = np.fromstring(image.colorImage, np.uint8).reshape( image.camera.colorHeight, image.camera.colorWidth, 3 )
				#img = cv2.imdecode(nparr, cv2.CV_LOAD_IMAGE_COLOR)
				header = cv.CreateMatHeader(image.camera.colorHeight, image.camera.colorWidth, cv.CV_8UC3)
				cv.SetData(header, image.colorImage, cv.CV_AUTOSTEP)
				cv2.imshow("img", np.asarray(header))
			else:
				print "Camera not found in server"
		except Ice.Exception, e:
			traceback.print_exc()
			print e
		return True





