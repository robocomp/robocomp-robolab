/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#include "jointmotorI.h"

JointMotorI::JointMotorI( Worker *_worker)
{
	worker = _worker;
	mutex = worker->w_mutex;                   //share worker mutex
	// Component initialization ...
}


JointMotorI::~JointMotorI()
{
	// Free component resources here
}

// Component functions, implementation

void JointMotorI::setPosition( const MotorGoalPosition & goalPos, const Ice::Current&)
{
	try
	{
		worker->setPosition( goalPos );
	}
	catch( RoboCompJointMotor::UnknownMotorException & ex)
	{
		throw ex;
	}
	catch( RoboCompJointMotor::HardwareFailedException & ex)
	{
		throw ex;
	}
}

void JointMotorI::setSyncPosition( const MotorGoalPositionList & goalPosList, const Ice::Current &)        
{
	try
	{
		worker->setSyncPosition( goalPosList);
	}
	catch( RoboCompJointMotor::UnknownMotorException & ex)
	{
		throw ex; 
	}
}

void JointMotorI::setSyncVelocity(const MotorGoalVelocityList & goalVelList, const Ice::Current &)
{
	try
	{
		worker->setSyncVelocity( goalVelList);
	}
	catch( RoboCompJointMotor::UnknownMotorException & ex)
	{
		throw ex; 
	}
}

void JointMotorI::setVelocity( const MotorGoalVelocity & goalVel, const Ice::Current&)
{
	try
	{
		worker->setReferenceVelocity(goalVel);
	}
	catch( RoboCompJointMotor::UnknownMotorException & ex)
	{
		throw ex; 
	}
}

//~ void JointMotorI::setReferenceVelocity( const MotorGoalVelocity & goalVel, const Ice::Current&)
//~ {
	//~ try
	//~ {
		//~ worker->setReferenceVelocity( goalVel);
	//~ }
	//~ catch( RoboCompJointMotor::UnknownMotorException & ex)
	//~ {
		//~ throw ex; 
	//~ }
//~ }

MotorParams JointMotorI::getMotorParams( const ::std::string& motor , const Ice::Current &)
{
	MotorParams mp;

	try
	{
		worker->getMotorParams( QString::fromStdString( motor ) , mp);
		return mp;
	}
	catch( RoboCompJointMotor::UnknownMotorException & ex)
	{
		throw ex;
	}
}

BusParams JointMotorI::getBusParams( const Ice::Current & )
{
  return worker->getBusParams();
}

MotorState JointMotorI::getMotorState( const ::std::string & motor , const Ice::Current &)
{
	MotorState state;
	try
	{
		worker->getState( QString::fromStdString( motor ) , state);
		return state;
	}
	catch( RoboCompJointMotor::UnknownMotorException & ex)
	{
		throw ex;
	}
}

MotorStateMap JointMotorI::getMotorStateMap( const MotorList &motorList, const ::Ice::Current&)
{
	MotorStateMap stateMap;
	MotorState state;
	foreach(std::string motor, motorList)
	{
		try
		{
			worker->getState( QString::fromStdString( motor ) , state);
			stateMap[motor] = state ;
		}
		catch(RoboCompJointMotor::UnknownMotorException & ex)
		{
			throw ex;
		}
	}
	return stateMap;
}

MotorParamsList JointMotorI::getAllMotorParams( const ::Ice::Current& )
{
	MotorParamsList list(0);

	list = worker->getAllMotorParams();
	if( list.empty() == false )
		return list;
	else
	{
		RoboCompJointMotor::UnknownMotorException ex("JointMotor::getAllMotorParams - Empty List");
		throw ex;
	}
}

void JointMotorI::getAllMotorState(MotorStateMap &mstateMap, const ::Ice::Current&)
{
  mstateMap = worker->getAllMotorState();
  qDebug()<<mstateMap["dunkerdrv0"].p;
  qDebug()<<mstateMap["dunkerdrv1"].p;
  qDebug()<<mstateMap["dunkerdrv2"].p;
  qDebug()<<mstateMap["dunkerdrv3"].p;
}

void JointMotorI::setZeroPos(const std::string& name, const Ice::Current&)
{
	worker->setZeroPos(name);
}

void JointMotorI::setSyncZeroPos(const Ice::Current&)
{
	worker->setSyncZeroPos();
}
void JointMotorI::stopAllMotors(const Ice::Current&)
{
	RoboCompJointMotor::HardwareFailedException hfailed("Not implemented yet");
	throw hfailed;	
}
void JointMotorI::stopMotor(const ::std::string &motor, const Ice::Current& )
{
	RoboCompJointMotor::HardwareFailedException hfailed("Not implemented yet");
	throw hfailed;	
}
void JointMotorI::releaseBrakeAllMotors(const Ice::Current& )
{
	RoboCompJointMotor::HardwareFailedException hfailed("Not implemented yet");
	throw hfailed;	
}
void JointMotorI::releaseBrakeMotor(const ::std::string &motor, const Ice::Current& )
{
	RoboCompJointMotor::HardwareFailedException hfailed("Not implemented yet");
	throw hfailed;	
}
void JointMotorI::enableBrakeAllMotors(const Ice::Current& )
{
	RoboCompJointMotor::HardwareFailedException hfailed("Not implemented yet");
	throw hfailed;	
}
void JointMotorI::enableBrakeMotor(const ::std::string &motor, const Ice::Current& )
{
	RoboCompJointMotor::HardwareFailedException hfailed("Not implemented yet");
	throw hfailed;	
}

	