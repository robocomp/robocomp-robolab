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
	

preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"

Ice.loadSlice(preStr+"TrajectoryRobot2D.ice")
from RoboCompTrajectoryRobot2D import *

class TrajectoryRobot2DI(TrajectoryRobot2D):
	def __init__(self, worker):
		self.worker = worker

	def getState(self, c):
		return self.worker.getState()
	def goBackwards(self, target, c):
		return self.worker.goBackwards(target)
	def stop(self, c):
		return self.worker.stop()
	def goReferenced(self, target, xRef, zRef, threshold, c):
		return self.worker.goReferenced(target, xRef, zRef, threshold)
	def changeTarget(self, target, c):
		return self.worker.changeTarget(target)
	def go(self, target, c):
		return self.worker.go(target)
	def mapBasedTarget(self, parameters, c):
		return self.worker.mapBasedTarget(parameters)





