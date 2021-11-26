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

import sys, os, Ice

ROBOCOMP = ''
try:
    ROBOCOMP = os.environ['ROBOCOMP']
except:
    print('$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
    ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
    raise RuntimeError('ROBOCOMP environment variable not set! Exiting.')


Ice.loadSlice("-I ./src/ --all ./src/JointMotorSimple.ice")

from RoboCompJointMotorSimple import *

class JointMotorSimpleI(JointMotorSimple):
    def __init__(self, worker):
        self.worker = worker


    def getMotorParams(self, motor, c):
        return self.worker.JointMotorSimple_getMotorParams(motor)

    def getMotorState(self, motor, c):
        return self.worker.JointMotorSimple_getMotorState(motor)

    def setPosition(self, name, goal, c):
        return self.worker.JointMotorSimple_setPosition(name, goal)

    def setVelocity(self, name, goal, c):
        return self.worker.JointMotorSimple_setVelocity(name, goal)

    def setZeroPos(self, name, c):
        return self.worker.JointMotorSimple_setZeroPos(name)
