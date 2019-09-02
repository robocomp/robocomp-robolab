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
#include "imuI.h"

IMUI::IMUI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
}


IMUI::~IMUI()
{
}

void IMUI::resetImu(const Ice::Current&)
{
	worker->resetImu();
}

Gyroscope IMUI::getAngularVel(const Ice::Current&)
{
	return worker->getAngularVel();
}

Orientation IMUI::getOrientation(const Ice::Current&)
{
	return worker->getOrientation();
}

DataImu IMUI::getDataImu(const Ice::Current&)
{
	return worker->getDataImu();
}

Magnetic IMUI::getMagneticFields(const Ice::Current&)
{
	return worker->getMagneticFields();
}

Acceleration IMUI::getAcceleration(const Ice::Current&)
{
	return worker->getAcceleration();
}






