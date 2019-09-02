#
# Copyright (C) 2019 by Bartlomiej Kocot
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

import sys, os, traceback, time,Ice,math,copy

import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

from PySide import *
from genericworker import *

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.stop()
		self.differentialrobot_proxy = proxy_map["DifferentialRobotProxy"]
		self.mousePress = False
		self.x=225
		self.y=225
		self.setGeometry(50, 50, 500, 500)
		self.setWindowTitle("Joystick Simulator Controller")
		self.setStyleSheet("QMainWindow {background: 'white';}");
		self.show()
		self.Speed=0.0
		self.Rotation=0.0
		self.addJoystickImage()

	def setParams(self, params):
		return True
	def addJoystickImage(self):
		self.Circle = QtGui.QLabel(self)
		self.JoyStick = QtGui.QLabel(self)
		self.SpeedText = QtGui.QLabel(self)
		self.SpeedValue = QtGui.QLabel(self)
		self.RotationText = QtGui.QLabel(self)
		self.RotationValue = QtGui.QLabel(self)
		circlePixmap = QtGui.QPixmap('src/img/circle.png')
		joystickPixmap = QtGui.QPixmap('src/img/joystick.png')
		self.Circle.setPixmap(circlePixmap)
		self.Circle.resize(200,200)
		self.Circle.move(150, 150)
		self.Circle.show()
		self.JoyStick.setObjectName("JoyStick")
		self.JoyStick.setPixmap(joystickPixmap)
		self.JoyStick.resize(50,50)
		self.JoyStick.move(225,225)
		self.JoyStick.show()
		self.SpeedText.setText("Speed: ")
		self.SpeedText.move(400,20)
		self.SpeedText.show()
		self.RotationText.setText("Rotation: ")
		self.RotationText.move(400,40)
		self.RotationText.show()
		self.SpeedValue.setText("0")
		self.SpeedValue.move(450,20)
		self.SpeedValue.show()
		self.RotationValue.setText("0")
		self.RotationValue.move(465,40)
		self.RotationValue.show()
	def mousePressEvent(self, event):
		if self.distance(event.x()-25,event.y()-25,self.x,self.y)<=25:
			self.mousePress=True
	def mouseReleaseEvent(self, event):
		self.mousePress=False
		self.comeBack()
		self.x=225
		self.y=225
	def mouseMoveEvent(self,event):
		if self.mousePress == True:
			if self.distance(event.x()-25,event.y()-25,225,225) < 100:
				self.x=event.x()-25
				self.y=event.y()-25
				self.setPosition(self.x,self.y)
			else:
		    		sin,cos=self.trigAlpha(event.x()-25,event.y()-25)
		    		self.x,self.y=self.findPoint(cos,sin)
		    		self.setPosition(self.x,self.y)
	def setPosition(self,x,y):
		self.JoyStick.move(x,y)
		self.Speed=(225-y)*22
		self.Rotation=(x-225)*0.02
		self.SpeedValue.setText(str(self.Speed))
		self.RotationValue.setText(str(self.Rotation))
		self.differentialrobot_proxy.setSpeedBase(self.Speed, self.Rotation)
	def comeBack(self):
		self.JoyStick.move(225,225)
		self.Speed = 0
		self.Rotation = 0
		self.SpeedValue.setText(str(self.Speed))
		self.RotationValue.setText(str(self.Rotation))
		self.differentialrobot_proxy.setSpeedBase(self.Speed, self.Rotation)
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
	@QtCore.Slot()
	def compute(self):
		return True

