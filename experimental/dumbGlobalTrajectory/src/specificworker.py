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

from trajectoryrobot2dI import *

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)

		lines = open("/home/robocomp/robocomp/components/robocomp-ursus/files/navigation.graph").readlines()
		print lines[0]
		nodes = int(lines[0])

		##http://networkx.github.io/documentation/latest/tutorial/tutorial.html
		self.coordinates = {}
		self.G = nx.Graph()
		
		lines = lines[1:]
		for i in xrange(nodes):
			node = i+1
			print 'Add node', node
			self.G.add_node(node)
			line = lines[i]
			c = line.split('#')
			self.coordinates[node] = (float(c[0]), float(c[1]))
			print self.coordinates[node]
			
		lines = lines[nodes:]
		for line in lines:
			print line
			src, dst = line.split('#')
			src = int(src)
			dst = int(dst)
			print src, dst
			dist = np.linalg.norm(np.array([abs(self.coordinates[src][0]-self.coordinates[src][0]), abs(self.coordinates[src][1]-self.coordinates[dst][1])]))
			self.G.add_edge(src,dst)
			self.G.add_edge(dst,src)
			self.G[src][dst]['distance'] = dist
			self.G[dst][src]['distance'] = dist



	def setParams(self, params):
		#try:
		#	par = params["InnerModelPath"]
		#	innermodel_path=par.value
		#	innermodel = InnerModel(innermodel_path)
		#except:
		#	traceback.print_exc()
		#	print "Error reading config params"
		return True

	@QtCore.Slot()
	def compute(self):
		#print 'SpecificWorker.compute...'
		#try:
		#	self.differentialrobot_proxy.setSpeedBase(100, 0)
		#except Ice.Exception, e:
		#	traceback.print_exc()
		#	print e
		return True


	#
	# getState
	#
	def getState(self):
		ret = NavState()
		#
		# YOUR CODE HERE
		#
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
		print 'set STOP'
		self.omnirobot_proxy.setSpeedBase(0,0,0)


	#
	# goReferenced
	#
	def goReferenced(self, target, xRef, zRef, threshold):
		ret = float()
		for c in self.coordinates:
		print(nx.shortest_path(self.G, source=1,target=16, weight='distance'))
		return ret


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





