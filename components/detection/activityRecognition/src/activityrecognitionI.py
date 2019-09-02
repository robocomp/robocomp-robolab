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

ice_ActivityRecognition = False
for p in icePaths:
	print 'Trying', p, 'to load ActivityRecognition.ice'
	if os.path.isfile(p+'/ActivityRecognition.ice'):
		print 'Using', p, 'to load ActivityRecognition.ice'
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"ActivityRecognition.ice"
		Ice.loadSlice(wholeStr)
		ice_ActivityRecognition = True
		break
if not ice_ActivityRecognition:
	print 'Couldn\'t load ActivityRecognition'
	sys.exit(-1)
from RoboCompActivityRecognition import *

class ActivityRecognitionI(ActivityRecognition):
	def __init__(self, worker):
		self.worker = worker

	def getCurrentActivity(self, c):
		return self.worker.getCurrentActivity()
	def addSkeleton(self, skeleton, c):
		return self.worker.addSkeleton(skeleton)
