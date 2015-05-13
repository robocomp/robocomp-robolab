#
# Copyright (C) 2015 by YOUR NAME HERE
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

import sys, os, Ice, traceback

import time

from PySide import *
from genericworker import *

# SpeechRecognition: https://pypi.python.org/pypi/SpeechRecognition/
import speech_recognition as sr

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	pass
if len(ROBOCOMP)<1:
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"ASRPublish.ice")
from RoboCompASRPublish import *



class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'
		try:
			# Optional Params: laguage. Example --> language = "en-US"
			#                  API key. Example --> key = "AIzaSyBOti4mM-6x9WDnZIjIeyEU21OpBXqWBgw"             For get a API key: http://www.chromium.org/developers/how-tos/api-keys
			# Google Speech Recognition API requires an API key. This library defaults to using 
			# one that was reverse engineered out of Chrome, but it is not recommended that you use this API key for anything other than personal or testing purposes.
			
			# Example to spanish
			# r = sr.Recognizer(language = "es-ES")
			r = sr.Recognizer(language = "es-ES")
			with sr.Microphone() as source:                # use the default microphone as the audio source
			  audio = r.adjust_for_ambient_noise(source) # listen for 1 second to calibrate the energy threshold for ambient noise levels
			  audio = r.listen(source)                   # now when we listen, the energy threshold is already set to a good value, and we can reliably catch speech right away
			  
			try:
			  command = r.recognize(audio)
			  print command
			  self.asrpublish.newText(command)
			except LookupError:                            # speech is unintelligible
			  self.asrpublish.newText("Error, could not understand audio")
		except Ice.Exception, e:
			traceback.print_exc()
			print e		