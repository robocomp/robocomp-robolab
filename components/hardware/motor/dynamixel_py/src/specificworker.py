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
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import sys, time
import numpy as np

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_LED = 25
ADDR_MX_CW_COMPLIANCE_MARGIN = 26
ADDR_MX_CW_COMPLIANCE_SLOPE = 27
ADDR_MX_CWW_COMPLIANCE_MARGIN = 28
ADDR_MX_CWW_COMPLIANCE_SLOPE = 29
ADDR_MX_MOVING_SPEED = 32
ADDR_MX_TORQUE_LIMIT = 34
ADDR_MX_PRESENT_SPEED = 38
ADDR_MX_PRESENT_LOAD = 40
ADDR_MX_PRESENT_VOLTAGE = 42
ADDR_MX_PRESENT_TEMPERATURE = 43
ADDR_MX_MOVING_STATUS = 46
ADDR_MX_PUNCH = 48

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 2
# Dynamixel ID : 1
BAUDRATE                    = 2000000            # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                # Value for enabling the torque
TORQUE_DISABLE              = 0                # Value for disabling the torque

MIN_POS_STEPS = 0
MAX_POS_STEPS = 1023                           # steps 0~1023 (0x3FF), and the unit is 0.29 [°]
MAX_POS_RANGE_RADS = 5.23
MIN_POS_RADS = -MAX_POS_RANGE_RADS/2
MAX_POS_RADS = MAX_POS_RANGE_RADS/2

# MAX SPEED
# 0 ~ 1,023(0x3FF) can be used, and the unit is about 0.111rpm.
# If it is set to 0, it means the maximum rpm of the motor is used without controlling the speed.
# If it is 1023, it is about 114rpm.
# For example, if it is set to 300, it is about 33.3 rpm.
MAX_SPEED_STEPS = 1023
MIN_SPEED_STEPS = 1
MAX_SPEED_RADSG = (np.pi*2.0/60)*0.111*1023
MIN_SPEED_RADSG = (np.pi*2.0/60)*0.111

# SET USB latency to 1ms
# echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
# $ echo ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"1\" > 99-dynamixelsdk-usb.rules # $ sudo cp ./99-dynamixelsdk-usb.rules /etc/udev/rules.d/
# $ sudo udevadm control --reload-rules
# $ sudo udevadm trigger --action=add
# $ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

        # Global vars
        self.current_pos = 0
        self.current_speed = 0
        self.current_temp = 0
        self.moving_status = False
        self.current_goal = None
        self.current_max_speed = None

        self.Period = 100
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True

    @QtCore.Slot()
    def compute(self):
        if self.current_goal:
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler,
                                                                           DXL_ID,
                                                                           ADDR_MX_MOVING_SPEED,
                                                                           self.current_max_speed)
            if not self.check_error("Goal max speed", dxl_comm_result, dxl_error):
                pass

            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler,
                                                                           DXL_ID,
                                                                           ADDR_MX_GOAL_POSITION,
                                                                           self.current_goal)
            if not self.check_error("Goal pos", dxl_comm_result, dxl_error):
                self.current_goal = None

        # POS
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler,
                                                                                            DXL_ID,
                                                                                            ADDR_MX_PRESENT_POSITION)
        if not self.check_error("Position", dxl_comm_result, dxl_error):
            self.current_pos = (MAX_POS_RANGE_RADS/MAX_POS_STEPS)*dxl_present_position - (MAX_POS_RANGE_RADS/2.0)
            #print("Pos (steps): ", dxl_present_position, " - rads", self.current_pos)

        # SPEED
        dxl_present_speed, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler,
                                                                                            DXL_ID,
                                                                                            ADDR_MX_PRESENT_SPEED)
        if not self.check_error("Speed", dxl_comm_result, dxl_error):
            if dxl_present_speed > MAX_SPEED_STEPS:
                self.current_speed = -(dxl_present_speed-MAX_SPEED_STEPS)*(MAX_SPEED_RADSG/MAX_SPEED_STEPS)
            else:
                self.current_speed = dxl_present_speed*(MAX_SPEED_RADSG/MAX_SPEED_STEPS)
            print("Speed  ", dxl_present_speed, self.current_speed)

        # TEMP
        dxl_present_temperature, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler,
                                                                                         DXL_ID,
                                                                                         ADDR_MX_PRESENT_TEMPERATURE)
        if not self.check_error("Temperature", dxl_comm_result, dxl_error):
            #print("Temp: ", dxl_present_temperature)
            self.current_temp = dxl_present_temperature

        # MOVING
        dxl_moving_status, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler,
                                                                                         DXL_ID,
                                                                                         ADDR_MX_MOVING_STATUS)
        if not self.check_error("Moving", dxl_comm_result, dxl_error):
            #print("Moving: ", dxl_moving_status)
            self.moving_status = dxl_moving_status

        return True

#####################################################################################################
    def check_error(self, var, dxl_comm_result, dxl_error):
        if dxl_comm_result != COMM_SUCCESS:
            print(var + " %s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return True
        elif dxl_error != 0:
            print(var + " %s" % self.packetHandler.getRxPacketError(dxl_error))
            return True
        else:
            return False

    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getMotorParams method from JointMotorSimple interface
    #
    def JointMotorSimple_getMotorParams(self, motor):
        ret = ifaces.RoboCompJointMotorSimple.MotorParams()
        return ret
    #
    # IMPLEMENTATION of getMotorState method from JointMotorSimple interface
    #
    def JointMotorSimple_getMotorState(self, motor):
        ret = ifaces.RoboCompJointMotorSimple.MotorState()
        ret.pos = self.current_pos      # rads -2.62 .. 2.62.
        ret.vel = self.current_speed    #rads / sg
        ret.power = 0
        ret.timeStamp = str(int(time.time()))
        ret.p = 0                       # steps
        ret.v = 0                       # steps / sg
        ret.isMoving = self.moving_status
        ret.temperature = self.current_temp
        return ret
    #
    # IMPLEMENTATION of setPosition method from JointMotorSimple interface
    #
    def JointMotorSimple_setPosition(self, name, goal):  # comes in radians -2.62 .. 2.62. Goes in STEPS 0..1023
        #print("Set position: ", goal)
        if goal.position <= MAX_POS_RADS and goal.position > MIN_POS_RADS and goal.maxSpeed >= 0 and goal.maxSpeed < 1024:
            self.current_goal = int((MAX_POS_STEPS/MAX_POS_RANGE_RADS)*goal.position + (MAX_POS_STEPS/2))
            self.current_max_speed = int(MAX_SPEED_STEPS/MAX_SPEED_RADSG*goal.maxSpeed)

    #
    # IMPLEMENTATION of setVelocity method from JointMotorSimple interface
    #
    def JointMotorSimple_setVelocity(self, name, goal):
    
        #
        # write your CODE here
        #
        pass

    #
    # IMPLEMENTATION of setZeroPos method from JointMotorSimple interface
    #
    def JointMotorSimple_setZeroPos(self, name):
    
        #
        # write your CODE here
        #
        pass


    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompJointMotorSimple you can use this types:
    # RoboCompJointMotorSimple.MotorState
    # RoboCompJointMotorSimple.MotorParams
    # RoboCompJointMotorSimple.MotorGoalPosition
    # RoboCompJointMotorSimple.MotorGoalVelocity


#####################################################################################################
    def startup_check(self):
        print(f"Testing RoboCompJointMotorSimple.MotorState from ifaces.RoboCompJointMotorSimple")
        test = ifaces.RoboCompJointMotorSimple.MotorState()
        print(f"Testing RoboCompJointMotorSimple.MotorParams from ifaces.RoboCompJointMotorSimple")
        test = ifaces.RoboCompJointMotorSimple.MotorParams()
        print(f"Testing RoboCompJointMotorSimple.MotorGoalPosition from ifaces.RoboCompJointMotorSimple")
        test = ifaces.RoboCompJointMotorSimple.MotorGoalPosition()
        print(f"Testing RoboCompJointMotorSimple.MotorGoalVelocity from ifaces.RoboCompJointMotorSimple")
        test = ifaces.RoboCompJointMotorSimple.MotorGoalVelocity()
        QTimer.singleShot(200, QApplication.instance().quit)

