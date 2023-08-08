# coding:UTF-8
import threading
import time
import serial
from serial import SerialException
'''
    Serial port configuration
'''


class SerialConfig:
    # ports
    portName = ''

    # baud
    baud = 9600

'''
Models of devices
'''


class DeviceModel:
    # Name of the device
    deviceName = "My device"  

    #Device ID
    ADDR = 0x50

    # Device Data Dictionary
    deviceData = {}

    # Is it stuck open?
    isOpen = False

    # serial port (computing)
    serialPort = None

    # Serial Port Configuration
    serialConfig = SerialConfig()

    # Update Trigger
    dataUpdateListener = ""

    # data parser
    dataProcessor = None

    # protocol parser
    protocolResolver = None

    def __init__(self, deviceName, protocolResolver, dataProcessor, dataUpdateListener):
        print("Initialising the device model")
        self.deviceName = deviceName
        self.protocolResolver = protocolResolver
        self.dataProcessor = dataProcessor
        self.dataUpdateListener = dataUpdateListener
		
        self.time = time.time()
        self.dictSensor = {}
        # _thread.start_new_thread(self.readDataTh, ("Data-Received-Thread", 10, ))
		

    def setDeviceData(self, key, value):
        """
        Setting device data
        :param key: Data key
        :param value: data value
        :return: No return
        """
        self.deviceData[key] = value
    
    def getDeviceDatas(self):
        return self.deviceData
		
    def getDeviceKeys(self):
        return self.deviceData.keys()
		
    def getDeviceData(self, key):
        """
        Obtaining device data
        :param key: Data key
        :return: Returns the data value, or None if the data key does not exist.
        """
        if ( key in self.deviceData):
            return self.deviceData[key]
        else:
            return None

    def removeDeviceData(self, key):
        """
        Deleting Device Data
        :param key: Data key
        :return: No return
        """
        del self.deviceData[key]

    def readDataTh(self, threadName, delay):
        """
        Read Data Thread
        :return:
        """
        print("Launch thread" + threadName)
        while self.isOpen:
            try:
                tlen = self.serialPort.inWaiting()
                if (tlen>0):
                    data = self.serialPort.read(tlen)
                    self.onDataReceived(data)
            except Exception as ex:
                print(ex)

    def openDevice(self):
        """
        Turn on the device
        :return: No return
        """

        # Close the port first.
        self.closeDevice()
        try:
            self.serialPort = serial.Serial(self.serialConfig.portName, self.serialConfig.baud, timeout=0.5)
            self.isOpen = True
            t = threading.Thread(target=self.readDataTh, args=("Data-Received-Thread",10,))          # Open a thread to receive data
            t.start()
        except SerialException:
            print("Opened " + self.serialConfig.portName + " with "+self.serialConfig.baud +" baud")

    def closeDevice(self):
        """
        Turn off the device
        :return: No return
        """
        print("Closing port...")
        self.isOpen = False
        if self.serialPort is not None:
            self.serialPort.close()
            print("The port is closed.")
        print("The device is off.")

    def onDataReceived(self, data):
        """
        When receiving data
        :param data: Data received
        :return: No return
        """
        if self.protocolResolver is not None:
            self.protocolResolver.passiveReceiveData(data, self)

    def get_int(self,dataBytes):
        """
        Convert byte arrays to int   = C# BitConverter.ToInt16
        :param dataBytes: arrays of bytes
        :return:
        """
        #return -(data & 0x8000) | (data & 0x7fff)
        return  int.from_bytes(dataBytes, "little", signed=True)

    def get_unint(self,dataBytes):
        """
        Convert byte arrays to uint
        :param data:
        :return:
        """
        return  int.from_bytes(dataBytes, "little")


    def sendData(self, data):
        """
        Sending data
        :return: Whether it was sent successfully or not
        """
        if self.protocolResolver is not None:
            self.protocolResolver.sendData(data, self)

    def readReg(self, regAddr,regCount):
        """
        Read registers
        :param regAddr: register address
        :param regCount: Number of registers
        :return:
        """
        if self.protocolResolver is not None:
            return self.protocolResolver.readReg(regAddr,regCount, self)
        else:
            return None

    def writeReg(self, regAddr,sValue):
        """
        Write Registers
        :param regAddr: register address
        :param sValue: write value
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.writeReg(regAddr,sValue, self)

    def unlock(self):
        """
        release
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.unlock(self)

    def save(self):
        """
        save configuration
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.save(self)

    def AccelerationCalibration(self):
        """
        Calibration of galvanometers
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.AccelerationCalibration(self)

    def BeginFiledCalibration(self):
        """
        Start magnetic field calibration
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.BeginFiledCalibration(self)

    def EndFiledCalibration(self):
        """
        End of magnetic field calibration
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.EndFiledCalibration(self)

    def sendProtocolData(self, data):
        """
        Send data with protocol
        :return:
        """
        if self.protocolResolver is not None:
            self.protocolResolver.sendData(data)


