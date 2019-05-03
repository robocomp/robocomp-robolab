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
import os
import threading
import time

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
		# try:
		#	self.innermodel = InnerModel(params["InnerModelPath"])
		# except:
		#	traceback.print_exc()
		#	print "Error reading config params"
		return True

	@QtCore.Slot()
	def compute(self):
		if self.text_queue.empty():
			pass
		else:
			text_to_say = self.text_queue.get()
			for rep in charsToAvoid:
				text_to_say = text_to_say.replace(rep, '\\' + rep)
			#				shellcommand = "echo " + text_to_say  + " | padsp festival --tts"
			shellcommand = "echo \"%s\" | padsp festival --tts --language spanish"%text_to_say
			print('Order: ' + text_to_say)
			print('Shell: "' + shellcommand + '"')
			os.system(shellcommand)

	#
	# isBusy
	#
	def isBusy(self):
		return 'festival' in os.subprocess.Popen(["ps", "ax"], stdout=os.subprocess.PIPE).communicate()[0]

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
