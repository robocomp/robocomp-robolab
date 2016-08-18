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

import sys, os, Ice, traceback, time, math

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
		self.generalState = NavState()
		self.timer.timeout.connect(self.compute)
		self.Period = 1000
		self.timer.start(self.Period)
		self.currentTarget = None
		lines = open("/home/robocomp/robocomp/components/robocomp-shelly/files/navigation.graph").readlines()
		nodes = int(lines[0])

		##http://networkx.github.io/documentation/latest/tutorial/tutorial.html
		self.knownCoordinates = {}
		self.G = nx.Graph()
		
		lines = lines[1:]
		for i in xrange(nodes):
			node = i
			self.G.add_node(node)
			line = lines[i]
			c = line.split('#')
			self.knownCoordinates[node] = np.array([float(c[0]), float(c[1])])
			
		lines = lines[nodes:]
		for line in lines:
			src, dst = line.split('#')
			src = int(src)
			dst = int(dst)
			dist = np.linalg.norm(self.knownCoordinates[src]-self.knownCoordinates[dst])
			self.G.add_edge(src, dst)
			self.G.add_edge(dst, src)
			self.G[src][dst]['distance'] = dist
			self.G[dst][src]['distance'] = dist



	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		if self.currentTarget != None:
			print 'got target', self
			if len(self.path) > 1:
				self.currentTarget.doRotation = False
				txRef = 0.
				tzRef = 0.
			else:
				txRef = self.receivedXRef
				tzRef = self.receivedZRef
			self.trajectoryrobot2d_proxy.goReferenced(self.currentTarget, txRef, tzRef, self.receivedThreshold)
			print 'send ('+str(self.currentTarget.x)+','+str(self.currentTarget.z)+') ['+str(txRef)+','+str(tzRef)+']  ', self.currentTarget.doRotation
			s = self.omnirobot_proxy.getBaseState()
			currentPose = np.array([s.correctedX, s.correctedZ])
			currentTarget = np.array([self.currentTarget.x, self.currentTarget.z])
			finalTarget = np.array([self.receivedTarget.x, self.receivedTarget.z])
			print np.linalg.norm(currentPose-currentTarget), self.receivedThreshold
			if np.linalg.norm(currentPose-currentTarget) < self.receivedThreshold:
				print 'got to waypoint'
				self.path = self.path[1:]
				try:
					self.currentTarget = copy.deepcopy(self.receivedTarget)
					self.currentTarget.x = self.path[0][0]
					self.currentTarget.z = self.path[0][1]
				except IndexError:
					self.currentTarget = None
				print "I'd go to ("+str(len(self.path))+")", self.currentTarget
				self.trajectoryrobot2d_proxy.goReferenced(self.currentTarget, txRef, tzRef, self.receivedThreshold)
				if np.linalg.norm(currentPose-finalTarget) < self.receivedThreshold:
					self.trajectoryrobot2d_proxy.stop()
					self.currentTarget = None
			else:
				print 'lalala', np.linalg.norm(currentPose-currentTarget), 'less than', self.receivedThreshold
			print 'distxxxance', np.linalg.norm(currentPose-currentTarget)
		else:
			#print 'idle'
			self.generalState.state = "idle"


	#
	# getState
	#
	def getState(self):
		return self.generalState


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
		print 'stoppppp'
		self.omnirobot_proxy.setSpeedBase(0,0,0)


	#
	# goReferenced
	#
	def goReferenced(self, target, xRef, zRef, threshold):
		ret = float()
		self.receivedTarget = target
		self.receivedXRef = xRef
		self.receivedZRef = zRef
		self.receivedThreshold = threshold
		if self.receivedThreshold < 50:
			self.receivedThreshold = 50

		baseState = self.omnirobot_proxy.getBaseState()
		baseState.x  = baseState.correctedX
		baseState.z  = baseState.correctedZ
		baseState.ry = baseState.correctedAlpha

		currentVector  = np.array([ baseState.x,  baseState.z])
		targetVector = np.array([self.receivedTarget.x, self.receivedTarget.z])

		nodeA = None
		aDist = np.linalg.norm(currentVector-targetVector)
		nodeB = None
		bDist = -1

		print 'XXXXXXXXXXXXXXXX'
		print 'targetvector', targetVector
		for c in self.knownCoordinates:
			dist = np.linalg.norm(self.knownCoordinates[c]-currentVector)
			if (dist < aDist and dist > self.receivedThreshold) and sameSigns(self.knownCoordinates[c], currentVector):
				nodeA = c
				aDist = dist
			dist = np.linalg.norm(self.knownCoordinates[c]-targetVector)
			if (dist < bDist or bDist < 0) and sameSigns(self.knownCoordinates[c], targetVector):
				nodeB = c
				bDist = dist

		if nodeA == None:
			path = []
		else:
			path = nx.shortest_path(self.G, source=nodeA, target=nodeB, weight='distance')

		print '----------------'
		print 'current pose', baseState.correctedX, baseState.correctedZ
		self.path = []
		for p in path:
			self.path.append(np.array([self.knownCoordinates[p][0], self.knownCoordinates[p][1]]))
		self.path.append(np.array([self.receivedTarget.x, self.receivedTarget.z]))
		print 'path1', self.path
		
		print '@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@'
		while len(self.path)>1:
			robotToFirst  = currentVector - self.path[0]
			robotToSecond = currentVector - self.path[1]
			print robotToFirst
			print robotToSecond
			c = np.dot(robotToFirst,robotToSecond)/np.linalg.norm(robotToFirst)/np.linalg.norm(robotToSecond)
			angle = np.arccos(np.clip(c, -1, 1))
			print angle
			if abs(angle) > math.pi/2:
				self.path = self.path[1:]
				print 'removed one'
			else:
				print 'continue'
				break
		print '############################################'


		print 'path2', self.path

		
		
		currentTargetT = copy.deepcopy(target)
		currentTargetT.x = self.path[0][0]
		currentTargetT.z = self.path[0][1]
		txRef = xRef
		tzRef = zRef
		if len(self.path) > 1:
			txRef = 0.
			tzRef = 0.
			diff = self.path[1]-self.path[0]
			currentTargetT.ry = math.atan2(diff[0], diff[1])
			currentTargetT.doRotation = True
		print 'scT1'
		if currentTargetT != self.currentTarget:
			self.currentTarget = currentTargetT
			self.trajectoryrobot2d_proxy.goReferenced(self.currentTarget, txRef, tzRef, self.receivedThreshold)
		print 'scT2'
		self.compute()
		print 'scT2'
		return np.linalg.norm(currentVector-targetVector)
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





