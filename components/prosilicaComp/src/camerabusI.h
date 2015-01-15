/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef CAMERABUSI_H
#define CAMERABUSI_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <CameraBus.h>

#include <config.h>
#include "worker.h"

using namespace RoboCompCameraBus;

class CameraBusI : public QObject , public virtual RoboCompCameraBus::CameraBus
{
Q_OBJECT
public:
	CameraBusI( Worker *_worker, QObject *parent = 0 );
	~CameraBusI();

	CameraParamsList getAllCameraParams( const Ice::Current & = Ice::Current());
	BusParams getBusParams( const Ice::Current & = Ice::Current());
	void getImage(const string& camera, const Format& frmt, Image& img, const Ice::Current & = Ice::Current());
	void getImageAndStates(const string& camera, const Format& frmt, Image& img, RoboCompJointMotor::MotorStateMap &hState, RoboCompDifferentialRobot::TBaseState& bState, const Ice::Current & = Ice::Current());
	void getSyncImages(const CameraList& cameras, const Format& frmt, bool all, ImageList& imglist, const Ice::Current & = Ice::Current());
	void getSyncImagesAndStates(const CameraList& cameras, const Format& frmt, bool all, ImageList& imglist, RoboCompJointMotor::MotorStateMap &hState, RoboCompDifferentialRobot::TBaseState& bState, const Ice::Current & = Ice::Current());


	QMutex *mutex;
private:

	Worker *worker;
public slots:


};

#endif
