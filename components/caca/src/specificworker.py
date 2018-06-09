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

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
sys.path.append('/opt/robocomp/lib')
import librobocomp_qmat
import librobocomp_osgviewer
import librobocomp_innermodel

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 50
		#self.timer.start(self.Period,)
		QtCore.QTimer.singleShot(10, self.compute);
		
	def setParams(self, params):
		try:
			print params['InnerModelPath']
			self.innermodel = librobocomp_innermodel.InnerModel(params["InnerModelPath"])
				
		except:
			traceback.print_exc()
			print "Error reading config params"
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'
	  
		camTomarca = self.innermodel.getTransformationMatrix("world", "marca1").invert()
		camTomarca.printmatrix("a")
		ang = camTomarca.extractAnglesR_min()
		ang.printvector("ang")
		print camTomarca[0,3], camTomarca[1,3], camTomarca[2,3]
		#insertar cam colgando de marca como cam1
		#camToWorld = self.innermodel.getTransformationMatrix("world", "cam1")
		#camToWorld.printmatrix("camToWorld")
		
		#a = librobocomp_qmat.QVec([0,0,0])
		#b = self.innermodel.transform("world", a , "cam1")
		##insertar cam colgando del mundo
		#b.printvector("d")
		
		

		# The API of python-innermodel is not exactly the same as the C++ version
		# self.innermodel.updateTransformValues("head_rot_tilt_pose", 0, 0, 0, 1.3, 0, 0)
		# z = librobocomp_qmat.QVec(3,0)
		# r = self.innermodel.transform("rgbd", z, "laser")
		# r.printvector("d")
		# print r[0], r[1], r[2]

		sys.exit(-1)


