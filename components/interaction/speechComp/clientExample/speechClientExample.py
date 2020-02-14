#!/usr/bin/env python
# -*- coding: utf-8 -*-

#    Copyright (C) 2010 by RoboLab
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


import sys, traceback, Ice, os, time

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	pass
if len(ROBOCOMP)<1:
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()

Ice.loadSlice(ROBOCOMP+'/Interfaces/Speech.ice')

import RoboCompSpeech

ic = None

class Client (Ice.Application):
	def run (self, argv):
		global ic
		status = 0
		self.shutdownOnInterrupt()
		ic = self.communicator()

		# Get connection config
		try:
			proxyString = ic.getProperties().getProperty('SpeechProxy')
		except:
			print 'Cannot get SpeechProxy property.'
			return

		# Remote object connection
		try:
			basePrx = self.communicator().stringToProxy(proxyString)
			self.speechPrx = RoboCompSpeech.SpeechPrx.checkedCast(basePrx)
		except:
			print 'Cannot connect to the remote object.'
			return

		#ordenes = ['I am speechComp, the component for speech synthesis']
		ordenes = ['hello hello hello hello hello hello hello hello hello hello hello hello hello hello bye', '']
		print ''
		for orden in ordenes:
			print 'Say: "' + orden + '"'
			c.speechPrx.say(orden, True)
			time.sleep(2)
		print '\nDone'

	def stop (self):
		if self.communicator():
			try:
				self.communicator().destroy()
			except:
				traceback.print_exc()
				status = 1

c = Client()
c.main(sys.argv)




