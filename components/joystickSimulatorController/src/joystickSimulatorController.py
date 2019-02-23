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
import sys,Ice,math,copy

# Ctrl+c handling
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

from PySide import *

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()
preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"CommonBehavior.ice")
import RoboCompCommonBehavior
Ice.loadSlice(preStr+"DifferentialRobot.ice")
import RoboCompDifferentialRobot

class MyGui(QtGui.QMainWindow):
    def __init__(self):
		QtGui.QMainWindow.__init__(self)
		self.setGeometry(50, 50, 500, 500)
		self.setWindowTitle("Joystick Simulator Controller")
		self.setStyleSheet("QMainWindow {background: 'white';}");
		self.show()
		self.mprx={}
		self.connect()
		self.differentialrobot_proxy = self.mprx["DifferentialRobotProxy"]
		self.Speed=0.0
		self.Rotation=0.0
		self.addJoystickImage()
    def connect(self):
	params = copy.deepcopy(sys.argv)
	if len(params) > 1:
		if not params[1].startswith('--Ice.Config='):
			params[1] = '--Ice.Config=' + params[1]
	else :# len(params) == 0:
		params.append('--Ice.Config=/etc/config')
	ic = Ice.initialize(params)
        status = 0
        try:
            # Remote object connection for DifferentialRobot
            try:
                proxyString = ic.getProperties().getProperty('DifferentialRobotProxy')
                try:
                    basePrx = ic.stringToProxy(proxyString)
                    differentialrobot_proxy = RoboCompDifferentialRobot.DifferentialRobotPrx.checkedCast(basePrx)
                    self.mprx["DifferentialRobotProxy"] = differentialrobot_proxy
                except Ice.Exception:
                    print 'Cannot connect to the remote object (DifferentialRobot)', proxyString
                    status = 1
            except Ice.Exception, e:
                status = 1
        except:
            status = 1
        if status == 1:
            print 'Cannot connect to the remote object (DifferentialRobot)'
	    self.hide()
	    ic.destroy()
	    sys.exit(0)
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
		if self.distance(x+event.x()-25,y+event.y()-25,225,225) < 100:
			x=x+event.x()-25
			y=y+event.y()-25
			mygui.setPosition(x,y)
		else:
		    sin,cos=self.trigAlpha(x+event.x()-25,y+event.y()-25) 
		    x,y=self.findPoint(cos,sin)
		    mygui.setPosition(x,y)
	return super(MouseDetector, self).eventFilter(obj, event)
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
mousePress = False
x=225
y=225
app = QtGui.QApplication(sys.argv)
mouseFilter = MouseDetector()
app.installEventFilter(mouseFilter)
mygui = MyGui()
sys.exit(app.exec_())
