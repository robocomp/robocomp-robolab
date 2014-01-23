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
#include "commonheadI.h"

CommonHeadI::CommonHeadI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
	// Component initialization...
}


CommonHeadI::~CommonHeadI()
{
	// Free component resources here
}

// Component functions, implementation
void CommonHeadI::resetHead(const Ice::Current&){
	worker->resetHead();
}

void CommonHeadI::stopHead(const Ice::Current&){
	worker->stopHead();
}

void CommonHeadI::setPanLeft(float pan, const Ice::Current&){
	worker->setPanLeft(pan);
}

void CommonHeadI::setPanRight(float pan, const Ice::Current&){
	worker->setPanRight(pan);
}

void CommonHeadI::setTilt(float tilt, const Ice::Current&){
	worker->setTilt(tilt);
}

void CommonHeadI::setNeck(float neck, const Ice::Current&){
	worker->setNeck(neck);
}

void CommonHeadI::saccadic2DLeft(float leftPan, float tilt, const Ice::Current&){
	worker->saccadic2DLeft(leftPan,tilt);
}

void CommonHeadI::saccadic2DRight(float rightPan, float tilt, const Ice::Current&){
	worker->saccadic2DRight(rightPan,tilt);
}

void CommonHeadI::saccadic3D(float leftPan, float rightPan, float tilt, const Ice::Current&){
	worker->saccadic3D(leftPan,rightPan,tilt);
}

void CommonHeadI::saccadic4D(float leftPan, float rightPan, float tilt, float neck, const Ice::Current&){
	worker->saccadic4D(leftPan,rightPan,tilt,neck);
}

void CommonHeadI::setNMotorsPosition(const RoboCompJointMotor::MotorGoalPositionList& listGoals, const Ice::Current&){
	worker->setNMotorsPosition(listGoals);
}

RoboCompCommonHead::THeadParams CommonHeadI::getHeadParams(const Ice::Current&){
	return worker->getHeadParams();
}

void CommonHeadI::getHeadState(RoboCompCommonHead::THeadState& hState, const Ice::Current&){
	worker->getHeadState(hState);
}

bool CommonHeadI::isMovingHead(const Ice::Current&){
	return worker->isMovingHead();
}


