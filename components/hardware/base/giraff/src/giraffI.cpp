/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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
#include "giraffI.h"

GiraffI::GiraffI(GenericWorker *_worker)
{
	worker = _worker;
}


GiraffI::~GiraffI()
{
}


float GiraffI::decTilt(const Ice::Current&)
{
	return worker->Giraff_decTilt();
}

RoboCompGiraff::Botones GiraffI::getBotonesState(const Ice::Current&)
{
	return worker->Giraff_getBotonesState();
}

float GiraffI::getTilt(const Ice::Current&)
{
	return worker->Giraff_getTilt();
}

float GiraffI::incTilt(const Ice::Current&)
{
	return worker->Giraff_incTilt();
}

void GiraffI::setTilt(float tilt, const Ice::Current&)
{
	worker->Giraff_setTilt(tilt);
}

