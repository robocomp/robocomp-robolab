#
#    Copyright (C) 2025 by YOUR NAME HERE
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

import sys, os, Ice

ROBOCOMP = ''
try:
    ROBOCOMP = os.environ['ROBOCOMP']
except:
    print('$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
    ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
    raise RuntimeError('ROBOCOMP environment variable not set! Exiting.')


Ice.loadSlice("-I ./generated/ --all ./generated/DifferentialRobot.ice")

from RoboCompDifferentialRobot import *

class DifferentialRobotI(DifferentialRobot):
    def __init__(self, worker, id:str):
        self.worker = worker
        self.id = id


    def correctOdometer(self, x, z, alpha, ice):
        return getattr(self.worker, f"DifferentialRobot{self.id}_correctOdometer")(x, z, alpha)

    def getBasePose(self, ice):
        return getattr(self.worker, f"DifferentialRobot{self.id}_getBasePose")()

    def getBaseState(self, ice):
        return getattr(self.worker, f"DifferentialRobot{self.id}_getBaseState")()

    def resetOdometer(self, ice):
        return getattr(self.worker, f"DifferentialRobot{self.id}_resetOdometer")()

    def setOdometer(self, state, ice):
        return getattr(self.worker, f"DifferentialRobot{self.id}_setOdometer")(state)

    def setOdometerPose(self, x, z, alpha, ice):
        return getattr(self.worker, f"DifferentialRobot{self.id}_setOdometerPose")(x, z, alpha)

    def setSpeedBase(self, adv, rot, ice):
        return getattr(self.worker, f"DifferentialRobot{self.id}_setSpeedBase")(adv, rot)

    def stopBase(self, ice):
        return getattr(self.worker, f"DifferentialRobot{self.id}_stopBase")()
