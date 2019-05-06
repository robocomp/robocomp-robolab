# encoding: utf-8
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
import subprocess

try:
	from Queue import Queue
except ImportError:
	from queue import Queue

from genericworker import *

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

max_queue = 100
charsToAvoid = ["'", '"', '{', '}', '[', '<', '>', '(', ')', '&', '$', '|', '#']


class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 200
		self.timer.start(self.Period)
		self.text_queue = Queue(max_queue)

	def __del__(self):
		print('SpecificWorker destructor')

	def setParams(self, params):
		if "tts" in params:
			self._tts = params["tts"]
		else:
			self._tts = "festival"
		# try:
		#	self.innermodel = InnerModel(params["InnerModelPath"])
		# except:
		#	traceback.print_exc()
		#	print "Error reading config params"
		return True

	@QtCore.Slot()
	def compute(self):
		print(self.isBusy())
		if self.text_queue.empty():
			pass
		else:
			text_to_say = self.text_queue.get()
			for rep in charsToAvoid:
				text_to_say = text_to_say.replace(rep, '\\' + rep)
			#				shellcommand = "echo " + text_to_say  + " | padsp festival --tts"
			print(self._tts)
			if "festival" in self._tts:
				self._say_with_festival(text_to_say)
			elif "google" in self._tts:
				self._say_with_google(text_to_say)




	def _say_with_festival(self, text):
		shellcommand = "echo \"%s\" | iconv -f utf-8 -t iso-8859-1 | padsp festival --tts --language spanish" % text
		print('Order: ' + text)
		print('Shell: "' + shellcommand + '"')
		os.system(shellcommand)

	def _say_with_google(self, text):
		try:
			from libs.google_TTS import google_tts_say
			print("Talking with google %s" % text)
			google_tts_say(text.decode(encoding='utf-8'))
		except ImportError:
			print("\033[91m To use google TTS you need to install gTTS package and playsound\033[00m")
			print("\033[91m You can try to install it with pip install gTTS playsound\033[00m")



	#
	# isBusy
	#
	def isBusy(self):
		if "festival" in self._tts:
			return 'festival' in subprocess.Popen(["ps", "ax"], stdout=subprocess.PIPE).communicate()[0]
		elif "google" in self._tts:
			try:
				from libs.google_TTS import google_tts_busy
				google_tts_busy()
			except ImportError:
				print("\033[91m To use google TTS you need to install gTTS package and playsound\033[00m")
				print("\033[91m You can try to install it with pip install gTTS playsound\033[00m")


	#
	# say
	#
	def say(self, text, owerwrite):
		if owerwrite:
			os.system("killall -9 festival")
			os.system("killall -9 aplay")
			self.text_queue = Queue(max_queue)
		self.text_queue.put(text)
		return True
