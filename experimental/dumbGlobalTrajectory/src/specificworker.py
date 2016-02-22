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

import sys, os, Ice, traceback, time

from PySide import *
from genericworker import *

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'genericworker.py: ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"OmniRobot.ice")
from RoboCompOmniRobot import *
Ice.loadSlice(preStr+"TrajectoryRobot2D.ice")
from RoboCompTrajectoryRobot2D import *
Ice.loadSlice(preStr+"Laser.ice")
from RoboCompLaser import *


import networkx as nx
import numpy as np
import copy

from trajectoryrobot2dI import *

def sameSigns(a, b):
	if a[0]*b[0]>0 and a[1]*b[1]>0:
		return True
	return False

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
		self.currentTarget = None
		lines = open("/home/robocomp/robocomp/components/robocomp-ursus/files/navigation.graph").readlines()
		nodes = int(lines[0])

		##http://networkx.github.io/documentation/latest/tutorial/tutorial.html
		self.coordinates = {}
		self.G = nx.Graph()
		
		lines = lines[1:]
		for i in xrange(nodes):
			node = i+1
			self.G.add_node(node)
			line = lines[i]
			c = line.split('#')
			self.coordinates[node] = np.array([float(c[0]), float(c[1])])
			
		lines = lines[nodes:]
		for line in lines:
			src, dst = line.split('#')
			src = int(src)
			dst = int(dst)
			dist = np.linalg.norm(self.coordinates[src]-self.coordinates[dst])
			self.G.add_edge(src, dst)
			self.G.add_edge(dst, src)
			self.G[src][dst]['distance'] = dist
			self.G[dst][src]['distance'] = dist



	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		if self.currentTarget != None:
			print 'got target'
			s = self.omnirobot_proxy.getBaseState()
			sV = np.array([s.correctedX, s.correctedZ])
			sT = np.array([self.currentTarget.x, self.currentTarget.z])
			if np.linalg.norm(sV-sT) < 100:
				self.currentTarget = copy.deepcopy(self.target)
				self.currentTarget.x = self.coordinates[self.path[0]][0]
				self.currentTarget.z = self.coordinates[self.path[0]][1]
				if len(self.path) > 1:
					self.currentTarget.doRotation = False
					txRef = 0.
					tzRef = 0.
				else:
					txRef = self.xRef
					tzRef = self.zRef
				self.trajectoryrobot2d_proxy.goReferenced(self.currentTarget, txRef, tzRef, self.threshold)
				self.path = self.path[1:]

	#
	# getState
	#
	def getState(self):
		ret = NavState()
		return ret


	#
	# goBackwards
	#
	def goBackwards(self, target):
		return self.goReferenced(target, 0, 0, 0)


	#
	# stop
	#
	def stop(self):
		l = QtCore.QMutexLocker(self.mutex)
		self.state.state = "IDLE"
		self.omnirobot_proxy.setSpeedBase(0,0,0)


	#
	# goReferenced
	#
	def goReferenced(self, target, xRef, zRef, threshold):
		ret = float()
		self.target = target
		self.xRef = xRef
		self.zRef = zRef
		self.threshold = threshold
		self.state = self.omnirobot_proxy.getBaseState()
		self.state.x  = self.state.correctedX
		self.state.z  = self.state.correctedZ
		self.state.ry = self.state.correctedAlpha
		stateV  = np.array([ self.state.x,  self.state.z])
		targetV = np.array([self.target.x, self.target.z])

		a = None
		aDist = np.linalg.norm(stateV-targetV)
		b = None
		bDist = -1


		for c in self.coordinates:
			dist = np.linalg.norm(self.coordinates[c]-stateV)
			if (dist < aDist or aDist < 0) and sameSigns(self.coordinates[c], stateV):
				a = c
				aDist = dist
			dist = np.linalg.norm(self.coordinates[c]-targetV)
			if (dist < bDist or bDist < 0) and sameSigns(self.coordinates[c], targetV):
				b = c
				bDist = dist
		if a == None:
			self.path = []
		else:
			self.path = nx.shortest_path(self.G, source=a,target=b, weight='distance')

		print '----------------'
		print self.state.x, self.state.z
		for p in self.path:
			print self.coordinates[p][0], self.coordinates[p][1], p
		self.path.append(np.array([self.target.x, self.target.z]))
		print self.target.x, self.target.z

		
		self.currentTarget = copy.deepcopy(target)
		self.currentTarget.x = self.coordinates[self.path[0]][0]
		self.currentTarget.z = self.coordinates[self.path[0]][1]
		txRef = xRef
		tzRef = zRef
		if len(self.path) != 1:
			txRef = 0.
			tzRef = 0.
			self.currentTarget.doRotation = False
		self.trajectoryrobot2d_proxy.goReferenced(self.currentTarget, txRef, tzRef, self.threshold)


		return np.linalg.norm(stateV-targetV)
	#
	# changeTarget
	#
	def changeTarget(self, target):
		return self.goReferenced(target, 0, 0, 0)


	#
	# go
	#
	def go(self, target):
		return self.goReferenced(target, 0, 0, 0)


	#
	# mapBasedTarget
	#
	def mapBasedTarget(self, parameters):
		#
		# YOUR CODE HERE
		#
		pass





