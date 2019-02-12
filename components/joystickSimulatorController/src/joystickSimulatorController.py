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
        self.setGeometry(50,50,500,500)
        self.setWindowTitle("Joystick Simulator Controller")
	self.setStyleSheet("QMainWindow {background: 'white';}");
        self.show()
	self.addJoystickImage()
    def addJoystickImage(self):
	self.label = QtGui.QLabel(self)
	pixmap = QtGui.QPixmap('src/img/joystick.png')
	self.label.setObjectName("JoyStick")
	self.label.setPixmap(pixmap)
	self.label.resize(50,50)
	self.label.move(225,225)	
	self.label.show()
    def setPosition(self,x,y):
	self.label.move(x,y)
    def comeBack(self):
	self.label.move(225,225)

class MouseDetector(QtCore.QObject):
    def eventFilter(self, obj, event):
	global mousePress, mygui,x,y
        if obj.objectName() == "JoyStick":
	    if event.type() == QtCore.QEvent.Type.MouseButtonPress:
		mousePress = True
	    if event.type() == QtCore.QEvent.Type.MouseButtonRelease:
	    	mousePress = False
		mygui.comeBack()
	    if event.type() == QtCore.QEvent.Type.MouseMove and mousePress == True:
		print event.x(),event.y()
		if self.distance(x+event.x()-25,y+event.y()-25) < 100:
			x=x+event.x()-25
			y=y+event.y()-25
			mygui.setPosition(x,y)
        return super(MouseDetector, self).eventFilter(obj, event)
    def distance(self,x,y):
	result = (225-x)*(225-x)+(225-y)*(225-y)
	result = math.sqrt(result)
	return result
mousePress = False
x=225
y=225
app = QtGui.QApplication(sys.argv)
mouseFilter = MouseDetector()
app.installEventFilter(mouseFilter)
mygui = MyGui()
sys.exit(app.exec_())
