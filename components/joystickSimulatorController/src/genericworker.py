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

import sys, Ice, os
from PySide import *

class GenericWorker(QtGui.QMainWindow):
	kill = QtCore.Signal()


	def __init__(self, mprx):
		QtGui.QMainWindow.__init__(self)
		self.setGeometry(50, 50, 500, 500)
		self.setWindowTitle("Joystick Simulator Controller")
		self.setStyleSheet("QMainWindow {background: 'white';}");
		self.show()
		self.differentialrobot_proxy = mprx["DifferentialRobotProxy"]
		self.Speed=0.0
		self.Rotation=0.0
		self.addJoystickImage()
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


	@QtCore.Slot()
	def killYourSelf(self):
		rDebug("Killing myself")
		self.kill.emit()

	# \brief Change compute period
	# @param per Period in ms
	@QtCore.Slot(int)
	def setPeriod(self, p):
		print "Period changed", p
		Period = p
		timer.start(Period)
