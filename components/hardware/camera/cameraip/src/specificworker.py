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

import sys, os, traceback, time, requests, cv2
import numpy as np

from PySide import  QtCore, QtGui
from genericworker import *

class FPS():
	def __init__(self):
		self.start_time = time.time() # start time of the loop
		self.cfps = 0
		
	def printa(self):
		if (time.time() - self.start_time) > 1:
			print("FPS: ", self.cfps / (time.time() - self.start_time)) # FPS = 1 / time to process loop
			self.start_time = time.time()
			self.cfps = 0
		else:
			self.cfps += 1


class ReadIPStream:
	def __init__(self, url):
		self.stream = requests.get(url, stream=True)
		
	def read(self):
		bytes = ''
		for chunk in self.stream.iter_content(chunk_size=1024):
			bytes += chunk
			a = bytes.find(b'\xff\xd8')
			b = bytes.find(b'\xff\xd9')
			if a != -1 and b != -1:
				jpg = bytes[a:b+2]
				bytes = bytes[b+2:]
				if len(jpg) > 0:
					img = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
					return True, img

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 50
		self.timer.start(self.Period)

	def setParams(self, params):
		try:
			camera = params["Camera"]
			if camera == "webcam":
				self.cap = cv2.VideoCapture(0)
			else:
				self.cap = ReadIPStream(camera)
			self.fps = FPS()
		except:
			traceback.print_exc()
			print "Error reading config params"
			sys.exit()
		return True

	@QtCore.Slot()
	def compute(self):
		ret, self.frame = self.cap.read()
		if ret:
			img = TImage(self.frame.shape[1], self.frame.shape[0], 3, ())
			cv2.imshow('CameraIP', self.frame)
			self.fps.printa()
			

	####################################################################################################

	#
	# getImage
	#
	def getImage(self):
		
		img = TImage(self.frame.shape[1], self.frame.shape[0], 3, ())
		img.image = self.frame.data
		return img

