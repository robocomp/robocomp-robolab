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

import sys, os, Ice

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()

additionalPathStr = ''
icePaths = []
try:
	icePaths.append('/opt/robocomp/interfaces')
	SLICE_PATH = os.environ['SLICE_PATH'].split(':')
	for p in SLICE_PATH:
		icePaths.append(p)
		additionalPathStr += ' -I' + p + ' '
except:
	print 'SLICE_PATH environment variable was not exported. Using only the default paths'
	pass

ice_IMU = False
for p in icePaths:
	print 'Trying', p, 'to load IMU.ice'
	if os.path.isfile(p+'/IMU.ice'):
		print 'Using', p, 'to load IMU.ice'
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"IMU.ice"
		Ice.loadSlice(wholeStr)
		ice_IMU = True
		break
if not ice_IMU:
	print 'Couldn\'t load IMU'
	sys.exit(-1)
from RoboCompIMU import *
ice_IMUPub = False
for p in icePaths:
	print 'Trying', p, 'to load IMUPub.ice'
	if os.path.isfile(p+'/IMUPub.ice'):
		print 'Using', p, 'to load IMUPub.ice'
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"IMUPub.ice"
		Ice.loadSlice(wholeStr)
		ice_IMUPub = True
		break
if not ice_IMUPub:
	print 'Couldn\'t load IMUPub'
	sys.exit(-1)
from RoboCompIMUPub import *

class IMUI(IMU):
	def __init__(self, worker):
		self.worker = worker

	def resetImu(self, c):
		return self.worker.resetImu()
	def getAngularVel(self, c):
		return self.worker.getAngularVel()
	def getOrientation(self, c):
		return self.worker.getOrientation()
	def getDataImu(self, c):
		return self.worker.getDataImu()
	def getMagneticFields(self, c):
		return self.worker.getMagneticFields()
	def getAcceleration(self, c):
		return self.worker.getAcceleration()
