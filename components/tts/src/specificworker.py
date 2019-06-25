#!/usr/bin/python
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

from genericworker import *
GreetList = []
ByeList = []
import random


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)

	def __del__(self):
		print 'SpecificWorker destructor'

	def setParams(self, params):
		#try:
		#	self.innermodel = InnerModel(params["InnerModelPath"])
		#except:
		#	traceback.print_exc()
		#	print "Error reading config params"
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'
		#computeCODE
		#try:
		#	self.differentialrobot_proxy.setSpeedBase(100, 0)
		#except Ice.Exception, e:
		#	traceback.print_exc()
		#	print e

		# The API of python-innermodel is not exactly the same as the C++ version
		# self.innermodel.updateTransformValues("head_rot_tilt_pose", 0, 0, 0, 1.3, 0, 0)
		# z = librobocomp_qmat.QVec(3,0)
		# r = self.innermodel.transform("rgbd", z, "laser")
		# r.printvector("d")
		# print r[0], r[1], r[2]

		return True

# ====================================== Methods for TTS =====================================================================================
	def ponerEsp(self):
	    voices  = self.engine.getProperty('voices')
	    idvoice = 0

	    for voice in voices:
		idvoice+=1
		if voice.id == 'spanish':
		        print("void ID: ", voice.id)
			self.engine.setProperty('voice', voice.id)	
	#
	# say
	#
	def say(self, text):
		if text is None: 
			print("¿Qué quieres escuchar?")
			text = input()
		self.ponerEsp()
		self.engine.say(text)	
		self.engine.runAndWait()
		pass

# ======================================== Methods for Alternative Phrases ===================================================================
	def sayAlternativeGreet(self):
		global GreetList
		if list:
			frase = random.choice(GreetList)
		else:
			frase = "Hola"
		self.say(frase)

	def addGreet(self):
		global GreetList
		frase = input()
		GreetList.append(frase)

	def deleteGreet(self):
		global GreetList
		frase = input()
		if frase in GreetList:
			GreetList.remove(frase)
		else:
			print('La frase no se encuentra en la lista')

	def showGreet(self):
		print(*GreetList, sep = "\n")

	def sayAlternativeBye(self):
		global ByeList
		if list:
			frase = random.choice(GreetList)
		else:
			frase = "Hola"
		self.say(frase)

	def addBye(self):
		global ByeList
		frase = input()
		GreetList.append(frase)

	def deleteBye(self):
		global ByeList
		frase = input()
		if frase in GreetList:
			GreetList.remove(frase)
		else:
			print('La frase no se encuentra en la lista')

	def showBye(self):
		print(*ByeList, sep = "\n")

