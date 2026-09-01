#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2025 by Alejandro Torrejon Harto
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
from time import time
from rich.console import Console
from rich.text import Text
from genericworker import *
import interfaces as ifaces
import numpy as np
from collections import deque
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent))
import SVD48V
import os

console = Console(highlight=False)

try:
    import setproctitle
    setproctitle.setproctitle(os.path.basename(os.getcwd()))
except:
    pass

ODOMETRY_WINDOW_SIZE=5
LINEAR_VELOCITY_DEADBAND=2 #mm/s
ANGULAR_VELOCITY_DEADBAND=0.01 #rad/s

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        print("Iniciando base")
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = 10
        
        #variables de clase
        self.targetSpeed = np.array([[0.0], [0.0], [0.0]])
        self.oldTargetSpeed = np.array([[0.0], [0.0], [0.0]])
        self.driver = None
        self.joystickControl = False
        self.time_disble = time()
        self.time_emergency = time()
        self.time_move = time()


        self.last_buttons = {
            "block": 0,
            "stop": 0,
            "joystick_control": 0
        }


        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)


    ####################################################### IMPORTANT ################################################################################
    #please in the SVD48VBase.py file the handler will call the __del__ 
    #SIGNALS handler
    #def sigint_handler(*args):
    #    QtCore.QCoreApplication.quit()
    #    worker.__del__()
    ####################################################### IMPORTANT ###########################################################
    def __del__(self):
        print("Finalizando base")
        """Destructor"""
        self.timer.stop()
        self.driver.__del__()
        print("Base destruida")        

    def setParams(self, params):
        print("Cargando parametros:")
        try:

            # Common params
            baseType = params["baseType"]
            assert baseType in ["Differential", "Omnidirectional"], "You must specificate the base type Diferential/Omnidirectional"

            axesLength =  float(params["axesLength"])
            port = params["port"]
            self.maxLinSpeed = float(params["maxLinSpeed"])
            self.maxRotSpeed = float(params["maxRotSpeed"])
            maxCurrent = int(params["maxCurrent"])
            maxAcceleration = int(params["maxAcceleration"])
            maxDeceleration = int(params["maxDeceleration"])
            idDrivers = [int(params["idDriver1"])]
            wheelRadius = int(params["wheelRadius"])
            polePairs = int(params["polePairs"])

            if baseType == "Omnidirectional":
                # Omnidireccional params
                self.isOmni=True
                distAxes =  float(params["distAxes"])
                idDrivers.append(int(params["idDriver2"]))
                
                ''''MATRIZ DE CONVESION'''
                ll = 0.5*(distAxes + axesLength)
                self.m_wheels = np.array([  [-1.0,  1.0, ll],
                                            [ 1.0,  1.0, -ll], 
                                            [ 1.0,  1.0, ll],
                                            [-1.0,  1.0, -ll]])
                
                maxWheelSpeed = max(abs(self.m_wheels@np.array([self.maxLinSpeed, self.maxLinSpeed, self.maxRotSpeed])))
                self.inv_m_wheels = np.linalg.pinv(self.m_wheels)
                
            else:
                # Diferential params
                self.isOmni=False
                self.m_wheels = np.array([[-1, axesLength/2], [-1, -axesLength/2]])
                # self.m_wheels = np.array([[1, -axesLength/2], [1, axesLength/2]]) TODO CHANGE WHEELS ORIENTATION
                maxWheelSpeed = max(abs(self.m_wheels@np.array([self.maxLinSpeed, self.maxRotSpeed])))
                self.inv_m_wheels = np.linalg.inv(self.m_wheels)
                
            print(self.m_wheels)

            self.driver = SVD48V.SVD48V(port=port, IDs=idDrivers, wheelRadius=wheelRadius, maxSpeed=maxWheelSpeed,
                                        maxAcceleration=maxAcceleration, maxDeceleration=maxDeceleration, maxCurrent=maxCurrent, polePairs=polePairs, noSafe=bool(params["noSafe"]))  

            assert self.driver.get_enable(), "NO se conecto al driver o fallo uno de ellos , cerrando programa"
            self.oldOdometry = self.driver.get_position().flatten()
            #                      num rot * rot2mm
            self.maxOdometryDiff = 13.9999999 * (2 * np.pi * wheelRadius)



            self.showParams = QTimer(self)
            self.showParams.timeout.connect(self.driver.show_params)
            #self.showParams.start(1000)
            self.timer.start(self.Period)
            
            print("Base iniciada correctamente")

        except Exception as e:
            print("Error reading config params or start motor")
            print(e)
            exit(-1)
        return True

    def setAdvx(self, val):
        if abs(val)>self.maxLinSpeed:  
            print("AVISO SUPERADA LA VELOCIDAD MAXIMA",val,"CUANDO MAXIMA ES", self.maxLinSpeed)
        self.targetSpeed[0] = np.clip(val, -self.maxLinSpeed,  self.maxLinSpeed)
        
    def setAdvz(self, val):
        if abs(val)>self.maxLinSpeed: 
            print("AVISO SUPERADA LA VELOCIDAD MAXIMA",val,"CUANDO MAXIMA ES", self.maxLinSpeed) 
        self.targetSpeed[1] = np.clip(val, -self.maxLinSpeed,  self.maxLinSpeed)
        
    def setRot(self, val):
        if abs(val)>self.maxRotSpeed:  
            print("AVISO SUPERADA LA VELOCIDAD MAXIMA", val,"CUANDO MAXIMA ES", self.maxRotSpeed)
        self.targetSpeed[2] = np.clip(val, -self.maxRotSpeed,  self.maxRotSpeed)

    def calcOdometry(self, vx, vy, vrz, timestamp):
        if not (np.isfinite(vx) and np.isfinite(vy) and np.isfinite(vrz)):
            print(f"[FullPoseEstimationPub_newFullPose] WARNING: invalid pose, skipping. {vx} {vy} {vrz}")
            return np.array((0.0, 0.0, 0.0))

        # Calcular dt a partir del timestamp de la pose (viene en ms)
        if not hasattr(self, "last_timestamp"):
            self.last_timestamp = timestamp
            self.last_odometry = np.array([0.0, 0.0, 0.0])
            return self.last_odometry

        dt = (timestamp - self.last_timestamp) / 1000.0
        self.last_timestamp = timestamp

        if dt <= 0.0 or dt > 1.0:
            print("WARNING: anomalous dt=", dt, "skipping")
            return self.last_odometry

        # Ventana deslizante de velocidades
        if not hasattr(self, "velocity_window"):
            self.velocity_window = deque(maxlen=ODOMETRY_WINDOW_SIZE)

        self.velocity_window.append((vx, vy, vrz))

        if len(self.velocity_window) < ODOMETRY_WINDOW_SIZE:
            return self.last_odometry

        avg_vx = avg_vy = avg_vrz = 0.0
        for vx_i, vy_i, vrz_i in self.velocity_window:
            avg_vx += vx_i
            avg_vy += vy_i
            avg_vrz += vrz_i

        avg_vx /= ODOMETRY_WINDOW_SIZE
        avg_vy /= ODOMETRY_WINDOW_SIZE
        avg_vrz /= ODOMETRY_WINDOW_SIZE

        if abs(avg_vy) < LINEAR_VELOCITY_DEADBAND:
            avg_vy = 0.0
        if abs(avg_vx) < LINEAR_VELOCITY_DEADBAND:
            avg_vx = 0.0
        if abs(avg_vrz) < ANGULAR_VELOCITY_DEADBAND:
            avg_vrz = 0.0

        last_x = self.last_odometry[0]
        last_y = self.last_odometry[1]
        last_theta = self.last_odometry[2]

        theta = last_theta + avg_vrz * dt
        x = last_x + (avg_vx * np.cos(last_theta) - avg_vy * np.sin(last_theta)) * dt
        y = last_y + (avg_vx * np.sin(last_theta) + avg_vy * np.cos(last_theta)) * dt

        self.last_odometry = np.array([x, y, theta])

        # print("[FullPoseEstimationPub_newFullPose]")
        # print(f"  Raw vel    -> vy: {vy} | vx: {vx} | vrz: {vrz}")
        # print(f"  Avg vel    -> vy: {avg_vy} | vx: {avg_vx} | vrz: {avg_vrz}")
        # print(f"  dt         -> {dt} s")
        # print(f"  Last odom  -> x: {last_x} | y: {last_y} | theta: {last_theta}")
        print(f"  New odom   -> x: {x} | y: {y} | theta: {theta}")

        return self.last_odometry
    #######################################COMPUTE###########################################
    @QtCore.Slot()
    def compute(self):
        print(flush=True, end="")
        if self.driver.get_enable() and self.driver.get_safety():
            if  not np.array_equal(self.targetSpeed, self.oldTargetSpeed):
                print(f"\033[32mModificamos velocidades: {np.round(self.oldTargetSpeed, 5).tolist()} a {np.round(self.targetSpeed, 5).tolist()} \033[0m")
                if self.isOmni:
                    speeds = self.m_wheels@self.targetSpeed
                else:
                    speeds = self.m_wheels@self.targetSpeed[1:]
                    
                #print(f"Modificamos velocidadespos: {np.round(self.oldTargetSpeed, 5).tolist()} a {np.round(self.targetSpeed, 5).tolist()} ")
                self.oldTargetSpeed = np.copy(self.targetSpeed)
                #print(f"oldTargetSpeed: {np.round(self.oldTargetSpeed, 5).tolist()}")

                self.driver.set_speed(speeds)
                #print(f"post speed: {np.round(self.oldTargetSpeed, 5).tolist()}\n")
                    

                self.time_move = time()
            #si en un segundo no hay nuevo target se detiene
            elif time() - self.time_move > 5:
                print("No comand, Stoping ")
                self.OmniRobot_setSpeedBase(0, 0, 0) if self.isOmni else self.DifferentialRobot_setSpeedBase(0, 0)
                self.driver.set_speed([0]*4 if self.isOmni else [0]*2)
                print("rpm",self.driver.get_rpm())
                if np.all(np.isclose(a=self.driver.get_rpm(),b=0, atol=0.5)): 
                    print("No comand, Stoped ")
                    self.time_move = float("inf")
                    self.driver.disable_driver()
                    self.driver.enable_driver()
                    
                    
            ##############PUB ODOMETRY####################
            try:
                odometry = ifaces.RoboCompFullPoseEstimation.FullPoseEuler()
                odometry.timestamp = np.longlong(time()*1000)
                
                #Millimeters odometry wheels
                newOdometry = self.driver.get_position().flatten()
                velocity = self.driver.get_speed().flatten()
                diffOdometry = newOdometry-self.oldOdometry
                
                positive_mask = diffOdometry > self.maxOdometryDiff
                negative_mask = diffOdometry < -self.maxOdometryDiff
                if np.any(positive_mask) or np.any(negative_mask):
                    console.print(
                        Text(f"Pose singularity with {diffOdometry}, maximum {self.maxOdometryDiff}", "bright_yellow"))
                    diffOdometry[positive_mask] -= self.maxOdometryDiff
                    diffOdometry[negative_mask] += self.maxOdometryDiff
                    console.print(
                        Text(f"Pose changed to {diffOdometry}", "bright_yellow"))

                self.oldOdometry = newOdometry

                #Convert wheel odometry to base odomery
                velocity = self.inv_m_wheels@velocity
                diffOdometry = self.inv_m_wheels@diffOdometry
                
                #Fill publish
                odometry.x = diffOdometry[1] if diffOdometry.shape[0]==3 else 0
                odometry.y = diffOdometry[0]
                odometry.z = 0
                odometry.rx = 0
                odometry.ry = 0
                odometry.rz = diffOdometry[2] if diffOdometry.shape[0]==3 else diffOdometry[1]
                odometry.vx = velocity[1] if velocity.shape[0]==3 else 0
                odometry.vy = -velocity[0] if velocity.shape[0]==3 else velocity[0] 
                odometry.vz = 0
                odometry.vrx = 0
                odometry.vry = 0
                odometry.vrz = velocity[2] if velocity.shape[0]==3 else velocity[1]
                odometry.ax = 0
                odometry.ay = 0
                odometry.az = 0
                odometry.arx = 0
                odometry.ary = 0
                odometry.arz = 0
                odometry.adv = odometry.vy
                odometry.side = odometry.vx
                odometry.rot = odometry.vrz
                odometry.confidence = 0
                # print(velocity)
                # print(odometry)
                self.fullposeestimationpub_proxy.newFullPose(odometry)
                self.calcOdometry(odometry.vy, odometry.vx, odometry.vrz, odometry.timestamp)
            except Exception as e:
                console.print_exception(e)
                console.print(Text("Fault reading odometry", style="yellow"))
        return True

    def startup_check(self):
        print(f"Testing RoboCompDifferentialRobot.TMechParams from ifaces.RoboCompDifferentialRobot")
        test = ifaces.RoboCompDifferentialRobot.TMechParams()
        print(f"Testing RoboCompOmniRobot.TMechParams from ifaces.RoboCompOmniRobot")
        test = ifaces.RoboCompOmniRobot.TMechParams()
        print(f"Testing RoboCompJoystickAdapter.AxisParams from ifaces.RoboCompJoystickAdapter")
        test = ifaces.RoboCompJoystickAdapter.AxisParams()
        print(f"Testing RoboCompJoystickAdapter.ButtonParams from ifaces.RoboCompJoystickAdapter")
        test = ifaces.RoboCompJoystickAdapter.ButtonParams()
        print(f"Testing RoboCompJoystickAdapter.TData from ifaces.RoboCompJoystickAdapter")
        test = ifaces.RoboCompJoystickAdapter.TData()
        QTimer.singleShot(200, QApplication.instance().quit)



    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of correctOdometer method from DifferentialRobot interface
    #
    def DifferentialRobot_correctOdometer(self, x, z, alpha):
        if not self.isOmni:
            #
            # write your CODE here
            #
            pass


    #
    # IMPLEMENTATION of getBasePose method from DifferentialRobot interface
    #
    def DifferentialRobot_getBasePose(self):
        if not hasattr(self, "last_odometry"):
            return [0,0,0]
        return self.last_odometry.tolist()
    #
    # IMPLEMENTATION of getBaseState method from DifferentialRobot interface
    #
    def DifferentialRobot_getBaseState(self):
        state = ifaces.RoboCompGenericBase.TBaseState()
        if hasattr(self, "last_odometry"):
            state.x = self.last_odometry[0]
            state.z = self.last_odometry[1]
            state.alpha = self.last_odometry[2]
        return state
    #
    # IMPLEMENTATION of resetOdometer method from DifferentialRobot interface
    #
    def DifferentialRobot_resetOdometer(self):
        print("[resetOdometer]")
        self.last_odometry = np.array([0.0, 0.0, 0.0])
        self.last_timestamp = time() * 1000
        self.velocity_window = deque(maxlen=ODOMETRY_WINDOW_SIZE)


    #
    # IMPLEMENTATION of setOdometer method from DifferentialRobot interface
    #
    def DifferentialRobot_setOdometer(self, state):
        if not self.isOmni:
            #
            # write your CODE here
            #
            pass


    #
    # IMPLEMENTATION of setOdometerPose method from DifferentialRobot interface
    #
    def DifferentialRobot_setOdometerPose(self, x, z, alpha):
        if not self.isOmni:
            #
            # write your CODE here
            #
            pass


    #
    # IMPLEMENTATION of setSpeedBase method from DifferentialRobot interface
    #
    def DifferentialRobot_setSpeedBase(self, adv, rot):
        if not self.isOmni and not self.joystickControl:
            self.setAdvz(adv)
            self.setRot(rot)

    #
    # IMPLEMENTATION of stopBase method from DifferentialRobot interface
    #
    def DifferentialRobot_stopBase(self):
        if not self.isOmni:
            self.time_emergency =time()
            self.DifferentialRobot_setSpeedBase(0, 0)
            self.driver.emergency_stop()

    #
    # IMPLEMENTATION of correctOdometer method from OmniRobot interface
    #
    def OmniRobot_correctOdometer(self, x, z, alpha):
        if self.isOmni:
            #
            # write your CODE here
            #
            pass


    #
    # IMPLEMENTATION of getBasePose method from OmniRobot interface
    #
    def OmniRobot_getBasePose(self):
        return self.DifferentialRobot_getBasePose()
    #
    # IMPLEMENTATION of getBaseState method from OmniRobot interface
    #
    def OmniRobot_getBaseState(self):
        return self.DifferentialRobot_getBaseState()
    #
    # IMPLEMENTATION of resetOdometer method from OmniRobot interface
    #
    def OmniRobot_resetOdometer(self):
        self.DifferentialRobot_resetOdometer()


    #
    # IMPLEMENTATION of setOdometer method from OmniRobot interface
    #
    def OmniRobot_setOdometer(self, state):
        if self.isOmni:
            #
            # write your CODE here
            #
            pass


    #
    # IMPLEMENTATION of setOdometerPose method from OmniRobot interface
    #
    def OmniRobot_setOdometerPose(self, x, z, alpha):
        if self.isOmni:
            #
            # write your CODE here
            #
            pass


    #
    # IMPLEMENTATION of setSpeedBase method from OmniRobot interface
    #
    def OmniRobot_setSpeedBase(self, advx, advz, rot):
        if self.isOmni and not self.joystickControl:
            self.setAdvx(-advx)
            self.setAdvz(advz)
            self.setRot(rot)
        if not self.isOmni and not self.joystickControl:
            self.setAdvz(advz)
            self.setRot(rot)



    #
    # IMPLEMENTATION of stopBase method from OmniRobot interface
    #
    def OmniRobot_stopBase(self):
         if self.isOmni:
            self.time_emergency =time()
            self.OmniRobot_setSpeedBase(0, 0, 0)
            self.driver.emergency_stop()
    
    def reset_emergency_stop(self):
        if time()-self.time_emergency>1:
            self.OmniRobot_setSpeedBase(0, 0, 0) if self.isOmni else self.DifferentialRobot_setSpeedBase(0, 0)
            self.driver.reset_emergency_stop()



    # =============== Methods for Component SubscribesTo ================
    # ===================================================================

    #
    # SUBSCRIPTION to sendData method from JoystickAdapter interface
    #
    def JoystickAdapter_sendData(self, data):    
        #print(data)
        for b in data.buttons:
            if b.name == "block":
                if b.step != self.last_buttons["block"]:
                    self.last_buttons["block"] = b.step
                    if b.step == 1:
                        if self.driver.get_safety():
                            self.OmniRobot_stopBase() if self.isOmni else self.DifferentialRobot_stopBase()
                        else:
                            self.reset_emergency_stop()
                        self.joystickControl = False
            elif  b.name == "stop":
                if b.step != self.last_buttons["stop"]:
                    self.last_buttons["stop"] = b.step
                    if b.step == 1:
                        if self.driver.get_enable():
                            self.time_disble = time()
                            self.driver.disable_driver()
                        elif time()-self.time_disble > 1:
                            self.driver.enable_driver()
                        self.joystickControl = False
            elif b.name == "joystick_control":
                if b.step != self.last_buttons["joystick_control"]:
                    self.last_buttons["joystick_control"] = b.step
                    if b.step == 1:
                        self.joystickControl = not self.joystickControl
                        if not self.joystickControl:
                            self.OmniRobot_setSpeedBase(0, 0, 0) if self.isOmni else self.DifferentialRobot_setSpeedBase(0, 0)

                        print("Joystick control: ", self.joystickControl)
            else:
                pass#print(b.name, "PULASDOR NO AJUSTADO")
            
        if self.joystickControl:
            for a in  data.axes:
                #print(a.name, a.value)
                if a.name == "rotate":
                    self.setRot(a.value)
                elif  a.name == "advance":
                    self.setAdvz(a.value)
                elif a.name == "side":
                    self.setAdvx(a.value)
                else:
                    pass#print(a.name, "JOYSTICK NO AJUSTADO")

        


    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompFullPoseEstimationPub you can publish calling this methods:
    # RoboCompFullPoseEstimationPub.void self.fullposeestimationpub_proxy.newFullPose(RoboCompFullPoseEstimation.FullPoseEuler pose)

    ######################
    # From the RoboCompDifferentialRobot you can use this types:
    # ifaces.RoboCompDifferentialRobot.TMechParams

    ######################
    # From the RoboCompOmniRobot you can use this types:
    # ifaces.RoboCompOmniRobot.TMechParams

    ######################
    # From the RoboCompJoystickAdapter you can use this types:
    # ifaces.RoboCompJoystickAdapter.AxisParams
    # ifaces.RoboCompJoystickAdapter.ButtonParams
    # ifaces.RoboCompJoystickAdapter.TData


