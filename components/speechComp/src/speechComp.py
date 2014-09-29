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

## \mainpage RoboComp::speechComp
 #
 # \section intro_sec Introduction
 #
 # speechComp, speech synthesis from text using the Festival speech synthesis software.
 #
 # \section interface_sec Interface
 #
 # speechComp interface provide one method for converting text to speech.
 #
 # \section install_sec Installation
 #
 # \subsection install1_ssec Software depencences
 # speechComp need Festival. sudo aptitude install festival.
 #
 # \subsection install2_ssec Compile and install
 # cd Tools/speechComp
 # <br>
 # It's an python component so no need compilation.
 #
 # \section guide_sec User guide
 #
 # \subsection config_ssec Configuration file
 #
 # <p>
 # The configuration file speechComp/etc/config.
 # </p>
 #
 # \subsection execution_ssec Execution
 #
 # Just: "${PATH_TO_BINARY}/speechComp --Ice.Config=${PATH_TO_CONFIG_FILE}"
 #
 # \subsection running_ssec Once running
 #
 #
 #

import sys, traceback, Ice, subprocess, threading, time, Queue, os

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	pass
if len(ROBOCOMP)<1:
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()

Ice.loadSlice(ROBOCOMP+"/interfaces/Speech.ice")
Ice.loadSlice(ROBOCOMP+"/interfaces/CommonBehavior.ice")

import RoboCompSpeech
import RoboCompCommonBehavior

sleep_time = 0.1
max_queue = 100
charsToAvoid = ["'", '"', '{', '}', '[', '<', '>', '(', ')', '&', '$', '|', '#']

class SpeechHandler (threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		self.text_queue = Queue.Queue(max_queue)
		self.accessLock = threading.Lock()
		self.initTime = time.time()
	def run (self):
		while 1:
			self.accessLock.acquire()
			if self.text_queue.empty():
				self.accessLock.release()
				time.sleep(sleep_time)
			else:
				text_to_say = self.text_queue.get()
				self.accessLock.release()
				for rep in charsToAvoid:
					text_to_say = text_to_say.replace(rep, '\\'+rep)
				shellcommand = "echo " + text_to_say  + " | padsp festival --tts"
				print 'Order: ' + text_to_say
				print 'Shell: "' + shellcommand + '"'
				os.system(shellcommand)
	def put (self, new_text, force):
		self.accessLock.acquire()
		if force:
			os.system("killall -9 festival")
			os.system("killall -9 aplay")
			self.text_queue = Queue.Queue(max_queue)
		self.text_queue.put(new_text)
		self.accessLock.release()

	def getFreq(self):
		return 5

	def setFreq(self, freq):
		print "Setting freq of acpi"

	def timeAwake(self):
		timeA=int(time.time()-self.initTime)
		type(timeA)
		return timeA


	def killYourSelf(self):
		print "Killing acpi component"

	def getAttrList(self, communicator):
		#~ attrList = dict()
		print "Obteniendo property dict"
		propertydict=communicator().getProperties().getPropertiesForPrefix("")
		print len(propertydict)
		attrList= []
		for k, v in propertydict.iteritems():
			attr= RoboCompCommonBehavior.AttrPair()
			attr.name=k
			attr.value=v
			attrList.append(attr)
		return attrList

class SpeechI (RoboCompSpeech.Speech):
	def __init__(self,_handler):
		self.handler = _handler
	def say(self, text, force, current=None):
		print text
		try:
			self.handler.put(text, force)
		except:
			print 'Full queue.'
	def isBusy(self, current=None):
		return 'festival' in subprocess.Popen(["ps", "ax"], stdout=subprocess.PIPE).communicate()[0]

class CommonBehaviorI (RoboCompCommonBehavior.CommonBehavior):
	def __init__(self, _handler, _communicator):
		self.handler = _handler
		self.communicator = _communicator
	def getFreq(self, current = None):
		self.handler.getFreq();
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

class Server (Ice.Application):
	def run (self, argv):
		status = 0;
		try:
			self.shutdownOnInterrupt()
			handler= SpeechHandler()
			handler.start()
			adapter = self.communicator().createObjectAdapter('SpeechComp')
			adapter.add(SpeechI(handler), self.communicator().stringToIdentity('speech'))
			adapter.add(CommonBehaviorI(handler, self.communicator), self.communicator().stringToIdentity('commonbehavior'))
			adapter.activate()
			self.communicator().waitForShutdown()
		except:
			traceback.print_exc()
			status = 1

		if self.communicator():
			try:
				self.communicator().destroy()
			except:
				traceback.print_exc()
				status = 1

Server( ).main(sys.argv)
