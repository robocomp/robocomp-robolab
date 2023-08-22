/*
 *    Copyright (C) 2023 by YOUR NAME HERE
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

// Ice includes
#include <Ice/Ice.h>
#include <IMU.h>

#include <config.h>
#include "genericworker.h"


class IMUI : public virtual RoboCompIMU::IMU
{
public:
	IMUI(GenericWorker *_worker);
	~IMUI();

	RoboCompIMU::Acceleration getAcceleration(const Ice::Current&);
	RoboCompIMU::Gyroscope getAngularVel(const Ice::Current&);
	RoboCompIMU::DataImu getDataImu(const Ice::Current&);
	RoboCompIMU::Magnetic getMagneticFields(const Ice::Current&);
	RoboCompIMU::Orientation getOrientation(const Ice::Current&);
	void resetImu(const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
