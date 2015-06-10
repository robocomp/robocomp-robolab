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

from PySide import *
from genericworker import *

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


from asrpublishI import *

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 4000
		self.timer.start(self.Period)
		
		self.showText = False
		self.showFullScreen()
		
		pathImage = 'imagesRobot/'
		self.ui.label_photoRobot.setPixmap(pathImage+'robot3.png')
		self.ui.label_photoHuman.setPixmap(pathImage+'persona1.png')
		
#		self.ui.label_textHuman.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)		
		self.ui.label_textHuman.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
		self.ui.label_textRobot.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)
		
		self.ui.label_textRobot.setText('Lorem ipsum dolor sit amet, consectetur adipricies sem Cras pulvin, maurisoicituding adipiscing, Lorem ipsum dolor sit amet, consect adipiscing elit, sed diam nonummy nibh euis ')
		

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
		print 'SpecificWorker.compute...'	
		try:
			if self.showText is True:
			      print self.text
			      if "Pablo" in self.text:
				self.ui.label_textRobot.setText("pablo, trabaja un poco")
				os.system('echo "Pablo, you dont work anithing. I miss you." | iconv -f utf-8 -t iso-8859-1 | festival --tts --language spanish')
			      else:
			          os.system('echo '+str(self.text)+' | iconv -f utf-8 -t iso-8859-1 | festival --tts --language spanish')
			          self.ui.label_textHuman.setText(str(self.text))

			      self.text = ''
			      self.showText = False
		except Ice.Exception, e:
			traceback.print_exc()
			print e
		
		return True


	#
	# newText
	#
	def newText(self, text):
		#
		# YOUR CODE HERE
		#
		if text != '':
		  self.showText = True
		  self.text = text
		else:
		  self.showText = False
		
		





