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

import sys, os, traceback, time
import numpy as np
import tensorflow as tf
from PySide import QtGui, QtCore
from genericworker import *


class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 50
		self.timer.start(self.Period)
		self.datafile_name = 'facial_database.npy'
		self.datafile_path = './'
		files = os.listdir(self.datafile_path)
		if (self.datafile_name not in files):
			print ("No dataset found. Creating a new empty numpy file")
			data = []
			np.save(os.path.join(self.datafile_path, self.datafile_name), data)
		self.data = np.load(os.path.join(self.datafile_path, self.datafile_name), allow_pickle=True)
		print ('Loaded the facial database')

	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'

		return True


	#
	# deleteLabel
	#
	def deleteLabel(self, faceLabel):
		#
		#implementCODE
		#
		pass


	#
	# getFaceLabels
	#
	def getFaceLabels(self, faces):
		#
		#implementCODE
		#
		faceNames = FaceLabels()
		return faceNames


	#
	# addNewFace
	#
	def addNewFace(self, faceImg):
		#
		#implementCODE
		#
		pass

