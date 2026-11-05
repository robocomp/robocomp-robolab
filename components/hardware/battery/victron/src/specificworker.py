#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2026 by Alejandro Torrejon Harto
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
from vedirect import Vedirect
import time, json
import os
import threading

console = Console(highlight=False)


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]
        
        self.on_warning = True
        self.warning_mode = 0
        self.warning_thread = threading.Thread(target=self.warning_function, daemon=True)
        self.warning_thread.start()

        if startup_check:
            self.startup_check()
        else:
            self.batteryStatus = ifaces.RoboCompBatteryStatus.TBattery()
            self.power_name = "battery"
            self.powerData = {}
            self.victron = Vedirect(serialport=configData["serialPort"], timeout=None,vMaxCharging=float(configData["vMaxCharging"]),
                                            vMax=float(configData["vMax"]), vMin=float(configData["vMin"]))
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""
        self.on_warning = False

    def warning_function(self):
        while self.on_warning:
            if self.warning_mode == 1:
                os.system("ffplay -nodisp -autoexit audio/Please_charge_me.mp3 > /dev/null")
            elif self.warning_mode == 2:
                os.system("ffplay -nodisp -autoexit audio/Hunger.mp3 > /dev/null")
            time.sleep(30)


    @QtCore.Slot()
    def compute(self):
        if self.victron is not None:
            # Leemos los datos del victron, obteniendo el diccionario
            victron_data_dict = self.victron.read_data()
            self.batteryStatus.voltage = int(victron_data_dict["V"])/1000
            self.batteryStatus.current = int(victron_data_dict["I"])/1000
            self.batteryStatus.consumptionPerHour = int(victron_data_dict["H20"])
            
            #Totalmente cargado
            if victron_data_dict["CS"]=="5": 
                self.batteryStatus.state = ifaces.RoboCompBatteryStatus.BatteryStates.Charged 
                self.batteryStatus.percentage = 100

            #Descargandose/No conectado
            elif victron_data_dict["CS"]=="0": 
                self.batteryStatus.state = ifaces.RoboCompBatteryStatus.BatteryStates.Disconnected 
                self.batteryStatus.percentage = self.victron.calc_percentage(charger=False, voltage=self.batteryStatus.voltage)
                if self.batteryStatus.percentage < 15:
                    self.warning_mode = 2
                elif self.batteryStatus.percentage < 25:
                    self.warning_mode = 1

            #Cargandose
            else:  
                self.batteryStatus.state = ifaces.RoboCompBatteryStatus.BatteryStates.Charging
                self.batteryStatus.percentage = self.victron.calc_percentage(charger=True, voltage=self.batteryStatus.voltage)
                self.warning_ticks = 0
                
            self.powerData[self.power_name] = ifaces.RoboCompPower.PowerData(
                name=self.power_name,
                voltage=self.batteryStatus.voltage,
                current=self.batteryStatus.current,
                power=self.batteryStatus.voltage * self.batteryStatus.current,
                timestamp=int(time.time() * 1000)
            )

            print(self.batteryStatus)
            # Mostramos los datos por pantalla de forma bonita
            print("=========== Datos recibidos ===========")
            print(json.dumps(victron_data_dict, indent=4))

        return True

    def startup_check(self):
        print(f"Testing RoboCompBatteryStatus.TBattery from ifaces.RoboCompBatteryStatus")
        test = ifaces.RoboCompBatteryStatus.TBattery()
        print(f"Testing RoboCompPower.Power from ifaces.RoboCompPower")
        test = ifaces.RoboCompPower.Power()
        QTimer.singleShot(200, QApplication.instance().quit)




    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getBatteryState method from BatteryStatus interface
    #
    def BatteryStatus_getBatteryState(self):
        return self.batteryStatus

    #
    # IMPLEMENTATION of getPower method from Power interface
    #
    def Power_getPower(self, name):
        try:
            return self.powerData[name]
        except KeyError:
            raise ifaces.RoboCompPower.UnknownSourceException(name=name)

    #
    # IMPLEMENTATION of getPowers method from Power interface
    #
    def Power_getPowers(self):
        return ifaces.RoboCompPower.Powers(self.powerData.values())
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompBatteryStatus you can use this types:
    # ifaces.RoboCompBatteryStatus.TBattery

    ######################
    # From the RoboCompPower you can use this types:
    # ifaces.RoboCompPower.PowerData


