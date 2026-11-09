#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2026 by Alejandro Torrejón Harto
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

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import os

import json
from ina226 import INA226
from time import time

console = Console(highlight=False)

sys.path.append('../../include')

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]
        if startup_check:
            self.startup_check()
        else:
            self.powerSensor = {}
            self.batterySensor = None
            names = list(configData["ID"].keys())

            for name in names:
                self.powerSensor[name] = INA226(busnum=1, address=int(configData["ID"][name],16), max_expected_amps=20, shunt_ohms=0.002)
                # 16 samples x (1.1 ms bus + 1.1 ms shunt) = ~35 ms per measurement, below the compute period
                self.powerSensor[name].configure(avg_mode=INA226.AVG_16BIT,
                                                 bus_ct=INA226.VCT_1100us_BIT,
                                                 shunt_ct=INA226.VCT_1100us_BIT)

                if name == "battery":
                    self.batterySensor =  self.powerSensor[name]

            if self.batterySensor is not None:
                self.paramsBattery = [float(configData["Battery"]["max"]), float(configData["Battery"]["min"])]
                assert min(self.paramsBattery)>0, "Battery limits must be upper zero"
                assert self.paramsBattery[0]>self.paramsBattery[1], "Max battery must upper that min battery"

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""


    @QtCore.Slot()
    def compute(self):
        print(self.BatteryStatus_getBatteryState())
        return True

    def startup_check(self):
        print(f"Testing RoboCompBatteryStatus.TBattery from ifaces.RoboCompBatteryStatus")
        test = ifaces.RoboCompBatteryStatus.TBattery()
        print(f"Testing RoboCompPower.PowerData from ifaces.RoboCompPower")
        test = ifaces.RoboCompPower.PowerData()
        QTimer.singleShot(200, QApplication.instance().quit)

    def calc_percentage(self, voltage):
        return (((voltage - self.paramsBattery[1])*100))/ (self.paramsBattery[0]-self.paramsBattery[1])

    def _read_sensor(self, sensor):
        try:
            v = sensor.voltage()
            c = sensor.current() / 1000.0
            # The power register is unsigned; V*I keeps the charge/discharge sign of the current
            p = v * c
            return v, c, p
        except Exception as e:
            console.print(f"[I2C] Error reading INA226: {e}", style="red", markup=False)
            raise

    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getBatteryState method from BatteryStatus interface
    #
    def BatteryStatus_getBatteryState(self):
        ret = ifaces.RoboCompBatteryStatus.TBattery()
        if self.batterySensor is not None:
            ret.voltage, ret.current, ret.power = self._read_sensor(self.batterySensor)
            ret.percentage = self.calc_percentage(voltage=ret.voltage)
        else:
            console.print("No has Battery sensor", style="red", markup=False)
        return ret
    #
    # IMPLEMENTATION of getPower method from Power interface
    #
    def Power_getPower(self, name):
        sensor = self.powerSensor.get(name)
        if sensor is None:
            raise ifaces.RoboCompPower.UnknownSourceException(name=name)
        ret = ifaces.RoboCompPower.PowerData()
        ret.name = name
        ret.voltage, ret.current, ret.power = self._read_sensor(sensor)
        ret.timestamp = int(time() * 1000)
        return ret
    #
    # IMPLEMENTATION of getPowers method from Power interface
    #
    def Power_getPowers(self):
        powers = []
        for name, sensor in self.powerSensor.items():
            try:
                p = ifaces.RoboCompPower.PowerData()
                p.name = name
                p.voltage, p.current, p.power = self._read_sensor(sensor)
                p.timestamp = int(time() * 1000)
                powers.append(p)
            except Exception as e:
                console.print(f"Skipping power sensor '{name}' due to read error", style="red", markup=False)
                continue
        return powers
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompBatteryStatus you can use this types:
    # ifaces.RoboCompBatteryStatus.TBattery

    ######################
    # From the RoboCompPower you can use this types:
    # ifaces.RoboCompPower.PowerData


