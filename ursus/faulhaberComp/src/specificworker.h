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
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <canbus/faulhaberApi.h>
#include "servo.h"
/**
       \brief
       @author authorname
*/

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx, QObject *parent = 0);
	~SpecificWorker();

	void  setPosition(const MotorGoalPosition& goal);
	void  setVelocity(const MotorGoalVelocity& goal);
	void  setZeroPos(const std::string& name);
	void  setSyncPosition(const MotorGoalPositionList& listGoals);
	void  setSyncVelocity(const MotorGoalVelocityList& listGoals);
	void  setSyncZeroPos();
	MotorParams getMotorParams(const std::string& motor);
	MotorState getMotorState(const std::string& motor);
	MotorStateMap getMotorStateMap(const MotorList& mList);
	void  getAllMotorState(MotorStateMap& mstateMap);
	MotorParamsList getAllMotorParams();
	BusParams getBusParams();

	void setParams(RoboCompCommonBehavior::ParameterList params);
private:
 	void run();
	void initializeMotors();		 
	float truncatePosition(QString name,float position);
private:
	int External_to_internal_encoder(int valor_potenciometro,int ID);
	//Parameters
	RoboCompJointMotor::BusParams busParams;
	RoboCompJointMotor::MotorParamsList motorParamsList;
	QHash<QString, RoboCompJointMotor::MotorParams> mParams;
	//Exceptions
	RoboCompJointMotor::HardwareFailedException hFailed;
	RoboCompJointMotor::UnknownMotorException uFailed;
	//Motors
	FaulHaberApi *faulhaber;
	QHash<QString, Servo*> motorsName;
	QHash<int,Servo*> motorsId;
	QHash<QString,int> name2id;
	QMutex * memory_mutex;
};

#endif
