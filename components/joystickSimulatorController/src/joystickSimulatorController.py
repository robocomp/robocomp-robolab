#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
import sys,math

from PySide import *

class MyGui(QtGui.QMainWindow):
    def __init__(self):
        QtGui.QMainWindow.__init__(self)
        self.setGeometry(50,50,1000,500)
        self.setWindowTitle("Joystick Simulator Controller")
	self.setStyleSheet("QMainWindow {background: 'white';}");
        self.show()
	self.addJoystickImage()
    def addJoystickImage(self):
	self.leftJoystick = QtGui.QLabel(self)
	self.rightJoystick = QtGui.QLabel(self)
	pixmap = QtGui.QPixmap('src/img/joystick.png')
	self.leftJoystick.setObjectName("leftJoystick")
	self.rightJoystick.setObjectName("rightJoystick")
	self.leftJoystick.setPixmap(pixmap)
	self.rightJoystick.setPixmap(pixmap)
	self.leftJoystick.resize(50,50)
	self.rightJoystick.resize(50,50)
	self.leftJoystick.move(225,225)
	self.rightJoystick.move(725,225)
	self.leftJoystick.show()
	self.rightJoystick.show()
    def setPosition(self,joystickName):
	if joystickName == "leftJoystick":
	    self.leftJoystick.move(leftJoystickX,leftJoystickY)
	else:
	    self.rightJoystick.move(rightJoystickX,rightJoystickY)
    def comeBack(self,joystickName):
	if joystickName == "leftJoystick":
	    self.leftJoystick.move(225,225)
	else:
	    self.rightJoystick.move(725,225)

class MouseDetector(QtCore.QObject):
    def eventFilter(self, obj, event):
	global mousePress, mygui,leftJoystickX,leftJoystickY,rightJoystickX,rightJoystickY
        if obj.objectName() == "leftJoystick" or obj.objectName() == "rightJoystick":
	    if event.type() == QtCore.QEvent.Type.MouseButtonPress:
		mousePress = True
	    if event.type() == QtCore.QEvent.Type.MouseButtonRelease:
	    	mousePress = False
		mygui.comeBack(obj.objectName())
	    if event.type() == QtCore.QEvent.Type.MouseMove and mousePress == True:
		if self.distance(event.x(),event.y(),obj.objectName()) < 100:
		    if obj.objectName() == "leftJoystick":
			leftJoystickX=leftJoystickX+event.x()-25
			leftJoystickY=leftJoystickY+event.y()-25
		    else:
			rightJoystickX=rightJoystickX+event.x()-25
			rightJoystickY=rightJoystickY+event.y()-25
		    mygui.setPosition(obj.objectName())
        return super(MouseDetector, self).eventFilter(obj, event)
    def distance(self,x,y,joystickName):
	if joystickName == "leftJoystick":
	    x=leftJoystickX+x-25
	    y=leftJoystickY+y-25
	    result = (225-x)*(225-x)+(225-y)*(225-y)
	else:
	    x=rightJoystickX+x-25
	    y=rightJoystickY+y-25
	    result = (725-x)*(725-x)+(225-y)*(225-y)
	result = math.sqrt(result)
	return result
	
mousePress = False
leftJoystickX=225
leftJoystickY=225
rightJoystickX=725
rightJoystickY=225
app = QtGui.QApplication(sys.argv)
mouseFilter = MouseDetector()
app.installEventFilter(mouseFilter)
mygui = MyGui()
sys.exit(app.exec_())
