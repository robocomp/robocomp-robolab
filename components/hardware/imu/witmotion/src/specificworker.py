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
from genericworker import *
import interfaces as ifaces
import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver
import time

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 2000

        #Creating device
        self.device = deviceModel.DeviceModel(
        "JY901",
        WitProtocolResolver(),
        JY901SDataProcessor(),
        "51_0"
        )

        if startup_check:
            self.startup_check()
        else:
            pass
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""
        #Close the device
        self.device.closeDevice()


    def setParams(self, params):
        try:
            self.port = params["port"]
        except:
            print("Error reading config params")

        
        print("WitMotion starting...")
        #Device communication settings
        self.device.serialConfig.portName = self.port
        self.device.serialConfig.baud = 230400
        #Opening device
        self.device.openDevice() 
        self.readConfig(self.device)

        #enabled sensors 
        enableSensors = {"TIME":False, "ACC":True, "GYRO": True, "ANGLE": True , "MAG":True, "PORT":False,
                        "PRESS":False, "GPS": False, "VELOCITY": False, "QUATER": False, "GSA":False} 
        sensors = 0
        for i, value in enumerate(enableSensors.values()):
            sensors |= value << i  

        #Apply settings to the device
        #self.setConfig(self.device, sensors)

        return True


    @QtCore.Slot()
    def compute(self):
        print('IMU Data ', self.device.getDeviceDatas())
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



    def readConfig(self, device):
        """
        Example of reading configuration information
        :param device: Device model
        :return:
        """
        tVals = device.readReg(0x02,3)  #Read data content, return rate, communication rate
        if (len(tVals)>0):
            print("Return results:" + str(tVals))
        else:
            print("No return")
        tVals = device.readReg(0x1F,1)  # Read the installation direction and algorithm
        if (len(tVals)>0):
            print("Return results:" + str(tVals)) 
        else:
            print("No return")

    def setConfig(self, device, valEnableSensors=None):
        """
        Example setting configuration information
        :param device: Device model
        :return:
        """
        device.unlock()                             # unlock
        time.sleep(0.1)                             # Sleep 100ms
        device.writeReg(0x03, 0x0B)                 # Set the Output rate 200Hz
        time.sleep(0.1)                             # Sleep 100ms
        device.writeReg(0x1F, 0)                    # Set the bandwidth rate 256Hz
        time.sleep(0.1)                             # Sleep 100ms
        if valEnableSensors is not None: 
            device.writeReg(0x02, valEnableSensors) # Set enabled sensors 
        time.sleep(0.1)                             # Sleep 100ms
        #device.writeReg(0x04, 7)                    # Set device bautrate 230400bps
        #time.sleep(0.1)                             # Sleep 100ms
        device.save() 

    def AccelerationCalibration(self, device):
        """
        Acceleration calibration
        :param device: Device model
        :return:
        """
        device.AccelerationCalibration()                 # Acceleration calibration
        print("End of galvanometric calibration")

    def FiledCalibration(self, device):
        """
        Magnetic field calibration
        :param device: Device model
        :return:
        """
        device.BeginFiledCalibration()                   #  Starting field calibration
        if input("Please make one slow rotation around the XYZ axis respectively, and end the calibration (Y/N) when the three axes are completed?").lower()=="y":
            device.EndFiledCalibration()                 #  End field calibration
            print("End of magnetic field calibration")


    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getAcceleration method from IMU interface
    #
    def IMU_getAcceleration(self):
        ret = ifaces.RoboCompIMU.Acceleration()
        ret.XAcc = self.device.getDeviceData("accX")
        ret.YAcc = self.device.getDeviceData("accY")
        ret.ZAcc = self.device.getDeviceData("accZ")
        return ret
    #
    # IMPLEMENTATION of getAngularVel method from IMU interface
    #
    def IMU_getAngularVel(self):
        ret = ifaces.RoboCompIMU.Gyroscope()
        ret.XGyr =  self.device.getDeviceData("gyroX")
        ret.YGyr =  self.device.getDeviceData("gyroY")
        ret.ZGyr =  self.device.getDeviceData("gyroZ")
        return ret
    #
    # IMPLEMENTATION of getDataImu method from IMU interface
    #
    def IMU_getDataImu(self):
        ret = ifaces.RoboCompIMU.DataImu()
        ret.acc = self.IMU_getAcceleration()
        ret.gyro = self.IMU_getAngularVel()
        ret.mag = self.IMU_getMagneticFields()
        ret.rot = self.IMU_getOrientation()
        ret.temperature = self.device.getDeviceData("temperature")
        return ret
    #
    # IMPLEMENTATION of getMagneticFields method from IMU interface
    #
    def IMU_getMagneticFields(self):
        ret = ifaces.RoboCompIMU.Magnetic()
        ret.XMag = self.device.getDeviceData("magX")
        ret.YMag = self.device.getDeviceData("magY")
        ret.ZMag = self.device.getDeviceData("magZ")
        return ret
    #
    # IMPLEMENTATION of getOrientation method from IMU interface
    #
    def IMU_getOrientation(self):
        ret = ifaces.RoboCompIMU.Orientation()
        ret.Roll = self.device.getDeviceData("angleY")
        ret.Pitch = self.device.getDeviceData("angleX")
        ret.Yaw = self.device.getDeviceData("angleZ")
        return ret
    #
    # IMPLEMENTATION of resetImu method from IMU interface
    #
    def IMU_resetImu(self):
    
        #
        # write your CODE here
        #
        pass


    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompIMU you can use this types:
    # RoboCompIMU.Acceleration
    # RoboCompIMU.Gyroscope
    # RoboCompIMU.Magnetic
    # RoboCompIMU.Orientation
    # RoboCompIMU.DataImu


