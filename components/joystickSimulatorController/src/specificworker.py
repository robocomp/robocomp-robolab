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

import sys, os, traceback, time,math

from PySide import QtGui, QtCore
from genericworker import *

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
	
	def __init__(self, proxy_map):
		return
	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self,x,y):
		if self.distance(x,y,225,225) > 100:
		    sin,cos=self.trigAlpha(x,y) 
		    x,y=self.findPoint(cos,sin)
		return x,y
	def distance(self,x1,y1,x2,y2):
		result = (x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)
		result = math.sqrt(result)
		return result
    	def trigAlpha(self,x,y):
		vecA_X=100.0
		vecA_Y=0
		vecB_X=x-225.0
		vecB_Y=y-225.0
		vecA_length=math.sqrt(vecA_X*vecA_X+vecA_Y*vecA_Y)
		vecB_length=math.sqrt(vecB_X*vecB_X+vecB_Y*vecB_Y)
		cosAlpha=(vecA_X*vecB_X+vecA_Y*vecB_Y)/(vecA_length*vecB_length)
		sinAlpha=(vecA_X*vecB_Y-vecA_Y*vecB_X)/(vecA_length*vecB_length)
		return sinAlpha,cosAlpha
    	def findPoint(self,cos,sin):
		pointX=225+100*cos
		pointY=225+100*sin
		return pointX,pointY
