#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
# Copyright (C) 2020 by YOUR NAME HERE
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

import sys, Ice, os
from PySide2 import QtWidgets, QtCore

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except KeyError:
	print('$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
	ROBOCOMP = '/opt/robocomp'

preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ --all /opt/robocomp/interfaces/"
Ice.loadSlice(preStr+"CommonBehavior.ice")
import RoboCompCommonBehavior

additionalPathStr = ''
icePaths = [ '/opt/robocomp/interfaces' ]
try:
	SLICE_PATH = os.environ['SLICE_PATH'].split(':')
	for p in SLICE_PATH:
		icePaths.append(p)
		additionalPathStr += ' -I' + p + ' '
	icePaths.append('/opt/robocomp/interfaces')
except:
	print('SLICE_PATH environment variable was not exported. Using only the default paths')
	pass

ice_GenericBase = False
for p in icePaths:
	if os.path.isfile(p+'/GenericBase.ice'):
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"GenericBase.ice"
		Ice.loadSlice(wholeStr)
		ice_GenericBase = True
		break
if not ice_GenericBase:
	print('Couln\'t load GenericBase')
	sys.exit(-1)
from RoboCompGenericBase import *
ice_CameraRGBDSimplePub = False
for p in icePaths:
	if os.path.isfile(p+'/CameraRGBDSimplePub.ice'):
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"CameraRGBDSimplePub.ice"
		Ice.loadSlice(wholeStr)
		ice_CameraRGBDSimplePub = True
		break
if not ice_CameraRGBDSimplePub:
	print('Couln\'t load CameraRGBDSimplePub')
	sys.exit(-1)
from RoboCompCameraRGBDSimplePub import *
ice_CameraRGBDSimple = False
for p in icePaths:
	if os.path.isfile(p+'/CameraRGBDSimple.ice'):
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"CameraRGBDSimple.ice"
		Ice.loadSlice(wholeStr)
		ice_CameraRGBDSimple = True
		break
if not ice_CameraRGBDSimple:
	print('Couln\'t load CameraRGBDSimple')
	sys.exit(-1)
from RoboCompCameraRGBDSimple import *
ice_JointMotor = False
for p in icePaths:
	if os.path.isfile(p+'/JointMotor.ice'):
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"JointMotor.ice"
		Ice.loadSlice(wholeStr)
		ice_JointMotor = True
		break
if not ice_JointMotor:
	print('Couln\'t load JointMotor')
	sys.exit(-1)
from RoboCompJointMotor import *
ice_HumanCameraBody = False
for p in icePaths:
	if os.path.isfile(p+'/HumanCameraBody.ice'):
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"HumanCameraBody.ice"
		Ice.loadSlice(wholeStr)
		ice_HumanCameraBody = True
		break
if not ice_HumanCameraBody:
	print('Couln\'t load HumanCameraBody')
	sys.exit(-1)
from RoboCompHumanCameraBody import *
ice_RGBD = False
for p in icePaths:
	if os.path.isfile(p+'/RGBD.ice'):
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"RGBD.ice"
		Ice.loadSlice(wholeStr)
		ice_RGBD = True
		break
if not ice_RGBD:
	print('Couln\'t load RGBD')
	sys.exit(-1)
from RoboCompRGBD import *


from camerargbdsimpleI import *
from rgbdI import *


class GenericWorker(QtCore.QObject):

	kill = QtCore.Signal()

	def __init__(self, mprx):
		super(GenericWorker, self).__init__()


		self.camerargbdsimplepub_proxy = mprx["CameraRGBDSimplePubPub"]
		self.humancamerabody_proxy = mprx["HumanCameraBodyPub"]

		
		self.mutex = QtCore.QMutex(QtCore.QMutex.Recursive)
		self.Period = 30
		self.timer = QtCore.QTimer(self)


	@QtCore.Slot()
	def killYourSelf(self):
		rDebug("Killing myself")
		self.kill.emit()

	# \brief Change compute period
	# @param per Period in ms
	@QtCore.Slot(int)
	def setPeriod(self, p):
		print("Period changed", p)
		self.Period = p
		self.timer.start(self.Period)
