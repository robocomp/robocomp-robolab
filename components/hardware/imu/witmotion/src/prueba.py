# coding:UTF-8

import time, copy
import datetime
import platform
import struct
import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver


def almost_equal(a, b, tolerance=1e-5):
    if isinstance(a, list) and isinstance(b, list):
        return all(almost_equal(x, y, tolerance) for x, y in zip(a, b))
    return abs(a - b) < tolerance

def onUpdate(deviceModel):
    """
    ??????  Data update event
    :param deviceModel: ????    Device model
    :return:
    """
    timestamp = time.time()- device.time
    for k in device.getDeviceKeys():
        if not k in device.dictSensor and deviceModel.getDeviceData(k) is not None: 
            device.dictSensor[k] = [time.time(), deviceModel.getDeviceData(k)]
        elif  not almost_equal(device.dictSensor[k][1], deviceModel.getDeviceData(k)):
            print("time " + str(time.time()-device.dictSensor[k][0]) + " " + k + ": " + f"{float(deviceModel.getDeviceData(k)):.6f}")
            device.dictSensor[k] = [time.time(), deviceModel.getDeviceData(k)]
    device.time = time.time()


def readConfig(device):
    """
    ????????    Example of reading configuration information
    :param device: ???? Device model
    :return:
    """
    tVals = device.readReg(0x02,3)  #????????????????   Read data content, return rate, communication rate
    if (len(tVals)>0):
        print("Devuelve los resultados:" + str(tVals))
    else:
        print("Sin retorno")
    tVals = device.readReg(0x1F,1)  #?????????  Read the installation direction and algorithm
    if (len(tVals)>0):
        print("Devuelve los resultados:" + str(tVals)) 
    else:
        print("Sin retorno")

def setConfig(device, valEnableSensors=None):
    """
    ????????    Example setting configuration information
    :param device: ???? Device model
    :return:
    """
    device.unlock()                # ?? unlock
    time.sleep(0.1)                # ??100??    Sleep 100ms
    device.writeReg(0x03, 0x0B)       # ??????? Set the Output rate 200Hz
    time.sleep(0.1)                # ??100??    Sleep 100ms
    device.writeReg(0x1F, 0)       # ???????    Set the bandwidth rate 256Hz
    time.sleep(0.1)                # ??100??    Sleep 100ms
    if valEnableSensors is not None: device.writeReg(0x02, valEnableSensors)       # ??????? set sensors 530
    time.sleep(0.1)                # ??100??    Sleep 100ms
    device.writeReg(0x04, 7)       # ??????? set bautrate 230400bps
    time.sleep(0.1)                # ??100??    Sleep 100ms
    device.save() 

"""
?????????   Initialize a device model
"""

dic = {}

device = deviceModel.DeviceModel(
  "??JY901",
  WitProtocolResolver(),
  JY901SDataProcessor(),
  "51_0"
)
enableSensors = {"TIME":False, "ACC":True, "GYRO": False, "ANGLE": True , "MAG":True, "PORT":False,
                "PRESS":False, "GPS": False, "VELOCITY": False, "QUATER": True, "GSA":False} 

sensors = 0
for i, value in enumerate(enableSensors.values()):
    sensors |= value << i
device.serialConfig.portName = "/dev/ttyUSB0"

device.serialConfig.baud = 230400                     #?????  Set baud rate
device.openDevice() 
readConfig(device)
setConfig(device, sensors)
time.sleep(0.1)
readConfig(device)
#device.dataProcessor.onVarChanged.append(onUpdate)
#input()
for _ in range(10000):
    print(device.getDeviceDatas())
    time.sleep(0.005)
device.closeDevice()

