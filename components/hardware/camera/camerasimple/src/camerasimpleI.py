#
# Copyright (C) 2020 by YOUR NAME HERE
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

import sys, os, Ice

try:
	Ice.loadSlice('/opt/robocomp/interfaces/CameraSimple.ice')
	print('Interface CameraSimple.ice loaded from /opt/robocomp/interfaces')
except:
	try:
		Ice.loadSlice(os.environ['ROBOCOMP']+'/interfaces/CameraSimple.ice')
		print('Interface CameraSimple loaded from '+os.environ['ROBOCOMP']+'/interfaces')
	except:
		print('Couldn\'t load CameraSimple.ice interface')
		sys.exit(-1)
import RoboCompCameraSimple

class CameraSimpleI(RoboCompCameraSimple.CameraSimple):
	def __init__(self, worker):
		self.worker = worker

	def getImage(self, c):
		return self.worker.CameraSimple_getImage()
