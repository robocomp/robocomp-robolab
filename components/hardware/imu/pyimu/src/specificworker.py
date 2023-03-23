#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2023 by YOUR NAME HERE
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
from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console

import sys, os, traceback, time

from genericworker import *
import interfaces as ifaces

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map,startup_check=False):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.imu = ifaces.RoboCompIMU.DataImu()
		self.Period = 500
		if startup_check:
			self.startup_check()
		else:
			self.timer.timeout.connect(self.compute)
			self.timer.start(self.Period)
			print("Start with period: ", self.Period)

	def __del__(self):
		"""Destructor"""

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
			print(values)
			if len(values)>3:
				self.imu.rot.Yaw = float(values[0])
				self.imu.rot.Roll = float(values[1])
				self.imu.rot.Pitch = float(values[2])
				print ("Data(y,r,p):", self.imu.rot.Yaw, self.imu.rot.Roll, self.imu.rot.Pitch)
				self.imupub_proxy.publish(self.imu)
			
		except Ice.Exception as e:
			traceback.print_exc()
			print(e)
		return True

	def startup_check(self):
		print(f"Testing RoboCompIMU.Acceleration from ifaces.RoboCompIMU")
		test = ifaces.RoboCompIMU.Acceleration()
		print(f"Testing RoboCompIMU.Gyroscope from ifaces.RoboCompIMU")
		test = ifaces.RoboCompIMU.Gyroscope()
		print(f"Testing RoboCompIMU.Magnetic from ifaces.RoboCompIMU")
		test = ifaces.RoboCompIMU.Magnetic()
		print(f"Testing RoboCompIMU.Orientation from ifaces.RoboCompIMU")
		test = ifaces.RoboCompIMU.Orientation()
		print(f"Testing RoboCompIMU.DataImu from ifaces.RoboCompIMU")
		test = ifaces.RoboCompIMU.DataImu()
		QTimer.singleShot(200, QApplication.instance().quit)

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
				return self.imu

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

    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompIMUPub you can publish calling this methods:
    # self.imupub_proxy.publish(...)

    ######################
    # From the RoboCompIMU you can use this types:
    # RoboCompIMU.Acceleration
    # RoboCompIMU.Gyroscope
    # RoboCompIMU.Magnetic
    # RoboCompIMU.Orientation
    # RoboCompIMU.DataImu
