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

import pyaudio
import wave


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


CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
RECORD_SECONDS = 5
WAVE_OUTPUT_FILENAME = "output.wav"


class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 10000
		self.timer.start(self.Period)
	def setParams(self, params):
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'
		try:
			p = pyaudio.PyAudio()

			stream = p.open(format=FORMAT,
					channels=CHANNELS,
					rate=RATE,
					input=True,
					frames_per_buffer=CHUNK)

			print("* recording")

			frames = []

			for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
			    data = stream.read(CHUNK)
			    frames.append(data)

			print("* done recording")

			stream.stop_stream()
			stream.close()
			p.terminate()

			wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
			wf.setnchannels(CHANNELS)
			wf.setsampwidth(p.get_sample_size(FORMAT))
			wf.setframerate(RATE)
			wf.writeframes(b''.join(frames))
			wf.close()

			r = sr.Recognizer()
			with sr.Microphone() as source:                # use the default microphone as the audio source
			    audio = r.adjust_for_ambient_noise(source) # listen for 1 second to calibrate the energy threshold for ambient noise levels

			with sr.WavFile("output.wav") as source:
			    audio = r.record(source)
			try:
			    command = r.recognize_google(audio, language="es-ES")    # recognize speech using Google Speech Recognition
			    print command
			    self.asrpublish.newText(command)
			except LookupError:                            # speech is unintelligible
			    print("Could not understand audio")
			    self.asrpublish.newText("Error, not understand audio")


		except Ice.Exception, e:
			traceback.print_exc()
			print e		
