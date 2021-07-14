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
#include "realsensefaceidI.h"

RealSenseFaceIDI::RealSenseFaceIDI(GenericWorker *_worker)
{
	worker = _worker;
}


RealSenseFaceIDI::~RealSenseFaceIDI()
{
}


RoboCompRealSenseFaceID::UserDataList RealSenseFaceIDI::authenticate(const Ice::Current&)
{
	return worker->RealSenseFaceID_authenticate();
}

bool RealSenseFaceIDI::enroll(std::string user, const Ice::Current&)
{
	return worker->RealSenseFaceID_enroll(user);
}

bool RealSenseFaceIDI::eraseAll(const Ice::Current&)
{
	return worker->RealSenseFaceID_eraseAll();
}

bool RealSenseFaceIDI::eraseUser(std::string user, const Ice::Current&)
{
	return worker->RealSenseFaceID_eraseUser(user);
}

RoboCompRealSenseFaceID::UserDataList RealSenseFaceIDI::getQueryUsers(const Ice::Current&)
{
	return worker->RealSenseFaceID_getQueryUsers();
}

bool RealSenseFaceIDI::startPreview(const Ice::Current&)
{
	return worker->RealSenseFaceID_startPreview();
}

bool RealSenseFaceIDI::stopPreview(const Ice::Current&)
{
	return worker->RealSenseFaceID_stopPreview();
}

