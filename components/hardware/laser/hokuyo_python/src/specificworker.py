#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
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
from PySide2.QtCore import QMutex, QMutexLocker
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import serial
from hokuyo.driver import hokuyo
from hokuyo.tools import serial_port
import numpy as np
import time

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 50
        # time
        self.start = time.time()
        self.counter = 0
        self.mutex = QMutex()
        self.ldata_write = []
        self.ldata_read = []

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        uart_port = '/dev/ttyACM1'
        uart_speed = 115200
        laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
        port = serial_port.SerialPort(laser_serial)

        self.laser = hokuyo.Hokuyo(port)
        res_on = self.laser.laser_on()
        print(res_on)

        return True

    @QtCore.Slot()
    def compute(self):

        # get_single_scan(self, start_step=START_STEP, stop_step=STOP_STEP, cluster_count=1):
        scan = self.laser.get_single_scan()

        self.ldata_write = []
        for angle, dist in scan.items():
            self.ldata_write.append(RoboCompLaser.TData(np.radians(angle), dist))

        if time.time() - self.start > 1:
            print("Freq -> ", self.counter, " Hz")
            self.counter = 0
            self.start = time.time()
        else:
            self.counter += 1

        locker = QMutexLocker(self.mutex)
        self.ldata_read, self.ldata_write = self.ldata_write, self.ldata_read

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getLaserAndBStateData method from Laser interface
    #
    def Laser_getLaserAndBStateData(self):
        ret = RoboCompLaser.TLaserData()
        bState = RoboCompGenericBase.TBaseState()
        return [ret, bState]
    #
    # IMPLEMENTATION of getLaserConfData method from Laser interface
    #
    def Laser_getLaserConfData(self):
        ret = RoboCompLaser.LaserConfData()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of getLaserData method from Laser interface
    #
    def Laser_getLaserData(self):
        locker = QMutexLocker(self.mutex)
        return self.ldata_read
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompLaser you can use this types:
    # RoboCompLaser.LaserConfData
    # RoboCompLaser.TData


