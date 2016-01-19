#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# Copyright (C) 2016 by YOUR NAME HERE
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

# \mainpage RoboComp::dumbTrajectoryRobot2DComp
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
# Just: "${PATH_TO_BINARY}/dumbTrajectoryRobot2DComp --Ice.Config=${PATH_TO_CONFIG_FILE}"
#
# \subsection running_ssec Once running
#
#
#

import sys, traceback, Ice, IceStorm, subprocess, threading, time, Queue, os, copy

# Ctrl+c handling
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

from PySide import *

from specificworker import *

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ -I/opt/robocomp/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"CommonBehavior.ice")
import RoboCompCommonBehavior
Ice.loadSlice(preStr+"OmniRobot.ice")
import RoboCompOmniRobot
Ice.loadSlice(preStr+"TrajectoryRobot2D.ice")
import RoboCompTrajectoryRobot2D
Ice.loadSlice(preStr+"Laser.ice")
import RoboCompLaser


class CommonBehaviorI(RoboCompCommonBehavior.CommonBehavior):
	def __init__(self, _handler, _communicator):
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
			return self.handler.getAttrList(self.communicator)
		except:
			print 'Problem getting getAttrList'
			traceback.print_exc()
			status = 1
			return



if __name__ == '__main__':
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
	try:

		# Remote object connection for TrajectoryRobot2D
		try:
			proxyString = ic.getProperties().getProperty('TrajectoryRobot2DProxy')
			try:
				basePrx = ic.stringToProxy(proxyString)
				trajectoryrobot2d_proxy = RoboCompTrajectoryRobot2D.TrajectoryRobot2DPrx.checkedCast(basePrx)
				mprx["TrajectoryRobot2DProxy"] = trajectoryrobot2d_proxy
			except Ice.Exception:
				print 'Cannot connect to the remote object (TrajectoryRobot2D)', proxyString
				#traceback.print_exc()
				status = 1
		except Ice.Exception, e:
			print e
			print 'Cannot get TrajectoryRobot2DProxy property.'
			status = 1


		# Remote object connection for OmniRobot
		try:
			proxyString = ic.getProperties().getProperty('OmniRobotProxy')
			try:
				basePrx = ic.stringToProxy(proxyString)
				omnirobot_proxy = RoboCompOmniRobot.OmniRobotPrx.checkedCast(basePrx)
				mprx["OmniRobotProxy"] = omnirobot_proxy
			except Ice.Exception:
				print 'Cannot connect to the remote object (OmniRobot)', proxyString
				#traceback.print_exc()
				status = 1
		except Ice.Exception, e:
			print e
			print 'Cannot get OmniRobotProxy property.'
			status = 1

	except:
			traceback.print_exc()
			status = 1


	if status == 0:
		worker = SpecificWorker(mprx)


		adapter = ic.createObjectAdapter('TrajectoryRobot2D')
		adapter.add(TrajectoryRobot2DI(worker), ic.stringToIdentity('trajectoryrobot2d'))
		adapter.activate()


#		adapter.add(CommonBehaviorI(<LOWER>I, ic), ic.stringToIdentity('commonbehavior'))

		app.exec_()

	if ic:
		try:
			ic.destroy()
		except:
			traceback.print_exc()
			status = 1
