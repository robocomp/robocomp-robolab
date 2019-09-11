#
# Copyright (C) 2018 by YOUR NAME HERE
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

import sys, os, traceback, time

from genericworker import *


class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.imu = DataImu()
		self.Period = 100
		self.timer.start(self.Period)
		print("Start with period: ", self.Period)

	def setParams(self, params):
		try:
			self.puerto = open(params["device"], "r")
			print ("Device opened:",)
		except IOError:
			print("Error opening serial port:", params["device"], "check device is connected")
			sys.exit(-1)
		return True

	@QtCore.Slot()
	def compute(self):
		print ('SpecificWorker.compute...')
		try:
			line = self.puerto.readline()
			values = line.strip().split(' ')
			self.imu.rot.Yaw = float(values[0])
			self.imu.rot.Roll = float(values[1])
			self.imu.rot.Pitch = float(values[2])
			print ("Data(y,r,p):", self.imu.rot.Yaw, self.imu.rot.Roll, self.imu.rot.Pitch)
			self.imupub_proxy.publish(self.imu)
			
		except Ice.Exception as e:
			traceback.print_exc()
			print(e)
		return True

# IMU implementation

		# resetImu
		#
		def resetImu(self):
				print("ERROR: not implemented yet")

		#
		# getAngularVel
		#
		def getAngularVel(self):
				ret = Gyroscope()
				return ret

		#
		# getOrientation
		#
		def getOrientation(self):
				ret = Orientation()
				return ret

		#
		# getDataImu
		#
		def getDataImu(self):
				return DataImu()

		#
		# getMagneticFields
		#
		def getMagneticFields(self):
				ret = Magnetic()
				return ret

		#
		# getAcceleration
		#
		def getAcceleration(self):
				ret = Acceleration()
