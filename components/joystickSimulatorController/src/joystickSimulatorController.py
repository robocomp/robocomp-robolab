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
#

# \mainpage RoboComp::joystickSimulatorController
#
# \section intro_sec Introduction
#
# Some information about the component...
#
# \section interface_sec Interface
#
# Descroption of the interface provided...
#
# \section install_sec Installation
#
# \subsection install1_ssec Software depencences
# Software dependences....
#
# \subsection install2_ssec Compile and install
# How to compile/install the component...
#
# \section guide_sec User guide
#
# \subsection config_ssec Configuration file
#
# <p>
# The configuration file...
# </p>
#
# \subsection execution_ssec Execution
#
# Just: "${PATH_TO_BINARY}/joystickSimulatorController --Ice.Config=${PATH_TO_CONFIG_FILE}"
#
# \subsection running_ssec Once running
#
#
#

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

from specificworker import *


class CommonBehaviorI(RoboCompCommonBehavior.CommonBehavior):
	def __init__(self, _handler):
		self.handler = _handler
		self.communicator = _communicator
	def getFreq(self, current = None):
		self.handler.getFreq()
	def setFreq(self, freq, current = None):
		self.handler.setFreq()
	def timeAwake(self, current = None):
		try:
			return self.handler.timeAwake()
		except:
			print 'Problem getting timeAwake'
	def killYourSelf(self, current = None):
		self.handler.killYourSelf()
	def getAttrList(self, current = None):
		try:
			return self.handler.getAttrList()
		except:
			print 'Problem getting getAttrList'
			traceback.print_exc()
			status = 1
			return

class MouseDetector(QtCore.QObject):
    def eventFilter(self, obj, event):
	global mousePress, mygui,x,y,worker
        if obj.objectName() == "JoyStick":
	    if event.type() == QtCore.QEvent.Type.MouseButtonPress:
		mousePress = True
	    if event.type() == QtCore.QEvent.Type.MouseButtonRelease:
	    	mousePress = False
		mygui.comeBack()
	    if event.type() == QtCore.QEvent.Type.MouseMove and mousePress == True:
		x,y=worker.compute(x+event.x()-25,y+event.y()-25)
		mygui.setPosition(x,y)
	return super(MouseDetector, self).eventFilter(obj, event)
if __name__ == '__main__':
	mousePress = False
	x=225
	y=225
	app = QtGui.QApplication(sys.argv)
	params = copy.deepcopy(sys.argv)
	if len(params) > 1:
		if not params[1].startswith('--Ice.Config='):
			params[1] = '--Ice.Config=' + params[1]
	elif len(params) == 1:
		params.append('--Ice.Config=config')
	ic = Ice.initialize(params)
	status = 0
	mprx = {}
	parameters = {}
	for i in ic.getProperties():
		parameters[str(i)] = str(ic.getProperties().getProperty(i))

	# Remote object connection for DifferentialRobot
	try:
		proxyString = ic.getProperties().getProperty('DifferentialRobotProxy')
		try:
			basePrx = ic.stringToProxy(proxyString)
			differentialrobot_proxy = RoboCompDifferentialRobot.DifferentialRobotPrx.checkedCast(basePrx)
			mprx["DifferentialRobotProxy"] = differentialrobot_proxy
		except Ice.Exception:
			print 'Cannot connect to the remote object (DifferentialRobot)', proxyString
			#traceback.print_exc()
			status = 1
	except Ice.Exception, e:
		print e
		print 'Cannot get DifferentialRobotProxy property.'
		status = 1

	if status == 0:
		worker = SpecificWorker(mprx)
		worker.setParams(parameters)
	else:
            print 'Cannot connect to the remote object (DifferentialRobot)'
	    ic.destroy()
	    sys.exit(0)
	mouseFilter = MouseDetector()
	app.installEventFilter(mouseFilter)
	mygui = GenericWorker(mprx)
	signal.signal(signal.SIGINT, signal.SIG_DFL)
	app.exec_()

	if ic:
		try:
			ic.destroy()
		except:
			traceback.print_exc()
			status = 1
