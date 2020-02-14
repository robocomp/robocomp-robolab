/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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
#ifndef IMU_H
#define IMU_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <IMU.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompIMU;

class IMUI : public QObject , public virtual RoboCompIMU::IMU
{
Q_OBJECT
public:
	IMUI( GenericWorker *_worker, QObject *parent = 0 );
	~IMUI();
	
	void resetImu(const Ice::Current&);
	Gyroscope getAngularVel(const Ice::Current&);
	Orientation getOrientation(const Ice::Current&);
	DataImu getDataImu(const Ice::Current&);
	Magnetic getMagneticFields(const Ice::Current&);
	Acceleration getAcceleration(const Ice::Current&);

	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:


};

#endif
