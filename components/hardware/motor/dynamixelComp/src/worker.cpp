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
#include "worker.h"
/**
* \brief Default constructor
*/
Worker::Worker(QObject *parent) : QThread(parent)
{
 	period        = 50;
	active        = false;
	handler       = NULL;
	w_mutex       = new QMutex(QMutex::Recursive);
	monitor_mutex = new QMutex(QMutex::Recursive);
	rDebug("||   CONSTRUCTOR WORKER OK   ||");
}
/**
* \brief Default destructor
*/
Worker::~Worker()
{
	v
	delete w_mutex;
	delete monitor_mutex;
}
/**
 * \brief Thread method
 */
void Worker::run( )
{
	qDebug()<<"Starting update loop.";
	forever
	{
		if (active)
		{
			handler->update();
			this->usleep(busParams.basicPeriod);
		}
	}
  //qDebug() << "Waiting for condition"; 
  //~ std::cout << "Worker.run() PARAMETERS_SET_WAIT_CONDITION->wait(mutex) 2" << std::endl;
  //~ PARAMETERS_SET_WAIT_CONDITION->wait(mutex);
  //~ std::cout << "Worker.run() PARAMETERS_SET_WAIT_CONDITION->wait(mutex) OK" << std::endl;

  //qDebug() << "Setting Parameters ";

}
///Common Behavior
void Worker::killYourSelf()
{
	rDebug("Killing myself");
	emit kill();
	exit(1);
}
/**
* \brief Change compute period
* @param per Period in ms
*/
void Worker::setPeriod(int p)
{
	rDebug("Period changed"+QString::number(p));
	period = p;
}

bool Worker::setParams(RoboCompCommonBehavior::ParameterList _params)
{
	QMutexLocker lock(monitor_mutex);
	active = false;

	qDebug() << "||   JOINT MOTOR --> Worker::setParams()  ||";

	if(_params["Dynamixel.Device"].value != busParams.device)
	{
		if(handler!= NULL)
			delete handler;
		busParams.device      = _params["Dynamixel.Device"].value;
		busParams.numMotors   = QString::fromStdString(_params["Dynamixel.NumMotors"].value).toInt();
		busParams.baudRate    = QString::fromStdString(_params["Dynamixel.BaudRate"].value).toInt();
		busParams.basicPeriod = QString::fromStdString(_params["Dynamixel.BasicPeriod"].value).toInt()*1000;

		for (int i=0; i<busParams.numMotors; i++)
		{
			std::string s     = QString::number(i).toStdString();
			RoboCompJointMotor::MotorParams mpar;
			mpar.name         = _params["Dynamixel.Params_" + s +".name"].value;
			mpar.busId        = QString::fromStdString(_params["Dynamixel.Params_" + s +".busId"].value).toUShort();
			mpar.invertedSign = QString::fromStdString(_params["Dynamixel.Params_" + s +".invertedSign"].value).contains("true");
			mpar.minPos       = QString::fromStdString(_params["Dynamixel.Params_" + s +".minPos"].value).toFloat();
			mpar.maxPos       = QString::fromStdString(_params["Dynamixel.Params_" + s +".maxPos"].value).toFloat();
			mpar.zeroPos      = QString::fromStdString(_params["Dynamixel.Params_" + s +".zeroPos"].value).toFloat();
			mpar.maxVelocity  = QString::fromStdString(_params["Dynamixel.Params_" + s +".maxVelocity"].value).toFloat();
			mpar.maxDegrees   = QString::fromStdString(_params["Dynamixel.Params_" + s +".maxDegrees"].value).toFloat();
			mpar.stepsRange   = QString::fromStdString(_params["Dynamixel.Params_" + s +".stepsRange"].value).toFloat();
			params.push_back(mpar);
		}
		//if config has the Dynamixel.SDK == FALSE, then we start the dynamixel serial handler (dynamixel_serial)
		if(not QString::fromStdString(_params["Dynamixel.SDK"].value).contains("true"))
		{
			handler = new Dynamixel(&busParams, &params, w_mutex);
			qDebug() << "||  Dynamixel::Worker::setParams()--> DYNAMIXEL SERIAL HANDLER SELECTED ||";
		}
		#if COMPILE_DYNAMIXEL == 1
		else
		{
			handler = new DynamixelSDK(&busParams, &params, w_mutex);
			qDebug() << "||  Dynamixel::Worker::setParams() DYNAMIXEL SDK HANDLER SELECTED ||";
		}
		#endif
		try
		{
			handler->initialize();
		}
		catch(QString &s)
		{
			qDebug()<<"Dynamixel error exception: "<<s;
			throw s;
		}
		this->start();
		active = true;
	}
	return active;
}


///
/// Servant methods
///

void Worker::setPosition( const RoboCompJointMotor::MotorGoalPosition & goalPosition )
{
	QMutexLocker ml(w_mutex);
	try
	{
		handler->setPosition( QString::fromStdString( goalPosition.name) , goalPosition.position, goalPosition.maxSpeed);
	}
	catch( const MotorHandlerErrorWritingToPortException &ex )
	{
		hFailed.what = std::string("Exception: JointMotorComp::Worker::setPosition::") + ex.what();
		throw hFailed;
	}
	catch( const MotorHandlerUnknownMotorException &ex )
	{
		hFailed.what = std::string("Exception: JointMotorComp::Worker::setPosition::") + ex.what();
		throw hFailed;
	}
}

void Worker::setVelocity( const RoboCompJointMotor::MotorGoalVelocity & goalVelocity )
{
	QMutexLocker ml(w_mutex);
	float velocity = goalVelocity.velocity;
	Servo *servo = handler->motors[QString::fromStdString(goalVelocity.name)];
	try
	{
		float position = servo->params.maxPos;
		if(goalVelocity.velocity == 0.f )
		{
			handler->getPosition(QString::fromStdString(goalVelocity.name),position);
 			velocity = 0.1;
		}
		else
			if(goalVelocity.velocity < 0.f )
				position = servo->params.minPos;
		
			
qDebug()<<"set velocity"<<QString::fromStdString(goalVelocity.name)<<fabs(velocity)<<position;
		handler->setPosition(QString::fromStdString( goalVelocity.name),position, fabs(velocity));
	}
	catch( const MotorHandlerErrorWritingToPortException &ex )
	{
		hFailed.what = std::string("Exception: JointMotorComp::Worker::setVelocity::") + ex.what();
		throw hFailed;
	}
	catch( const MotorHandlerUnknownMotorException &ex )
	{
		hFailed.what= std::string("Exception: JointMotorComp::Worker::setVelocity::") + ex.what();
		throw hFailed;
	}
}

void Worker::setSyncVelocity( const RoboCompJointMotor::MotorGoalVelocityList & goalVelList)
{
	qDebug()<<"setting sync velocity";
	QVector<Dynamixel::GoalPosition> hPositionList;
	float velocity = 0.1;
	
	QMutexLocker ml(w_mutex);
	
	for(uint i=0; i< goalVelList.size() ; i++)
	{
		QString motorName = QString::fromStdString( goalVelList[i].name);
		if ( handler->motors.contains( motorName )  == false )
		{
			RoboCompJointMotor::UnknownMotorException ex(goalVelList[i].name);
			throw ex;
		}
		else
		{
			Servo *servo = handler->motors[motorName];
			float position = servo->params.maxPos;
			velocity = goalVelList[i].velocity;
			if(goalVelList[i].velocity == 0.f )
			{
				handler->getPosition(QString::fromStdString(goalVelList[i].name),position);
				velocity = 0.1;
			}
			else if(goalVelList[i].velocity < 0.f)
			{
				position = servo->params.minPos;
				velocity = fabs(goalVelList[i].velocity);
			}
qDebug()<<"set sync velocity"<<QString::fromStdString(goalVelList[i].name)<<velocity<<position;
			Dynamixel::GoalPosition gp(motorName, servo->params.busId, position, velocity);
			hPositionList.append( gp );
		}
	}
	try
	{
		handler->setSyncPosition( hPositionList );
	}
	catch( QString &s)
	{
		RoboCompJointMotor::HardwareFailedException ex;
		ex.what = std::string("Exception: DynamixelComp::Worker::setSyncVelocity::") + s.toStdString();
		throw ex;
	}
}


void Worker::setReferenceVelocity( const RoboCompJointMotor::MotorGoalVelocity & goalVelocity )
{
	QString motorName = QString::fromStdString(goalVelocity.name);
	QMutexLocker ml(w_mutex);
	if (handler->motors.contains(motorName))
	{
		Servo *servo = handler->motors[motorName];
		printf("Worker::setReferenceVelocity() --> servo->rads2Steps( goalVelocity.velocity) = %d\n", servo->rads2Steps( goalVelocity.velocity));
		if (handler->setReferenceVelocity( servo->params.busId , servo->rads2Steps(goalVelocity.velocity)) == false)
		{
			RoboCompJointMotor::HardwareFailedException ex(goalVelocity.name);
			throw ex;
		}
	}
	else
	{
		RoboCompJointMotor::UnknownMotorException ex(goalVelocity.name);
		throw ex;
	}
}


void Worker::setSyncPosition( const RoboCompJointMotor::MotorGoalPositionList & goalPosList)
{
	QVector<Dynamixel::GoalPosition> hPositionList;
	QMutexLocker ml(w_mutex);
	
	for(uint i=0; i< goalPosList.size() ; i++)
	{
		QString motorName = QString::fromStdString( goalPosList[i].name);
		if ( handler->motors.contains( motorName )  == false )
		{
			RoboCompJointMotor::UnknownMotorException ex(goalPosList[i].name);
			throw ex;
		}
		else
		{
			Servo *servo = handler->motors[motorName];
			Dynamixel::GoalPosition gp(motorName, servo->params.busId, goalPosList[i].position, goalPosList[i].maxSpeed);
			hPositionList.append( gp );
		}
	}
	try
	{
		handler->setSyncPosition( hPositionList );
	}
	catch( QString &s)
	{
		RoboCompJointMotor::HardwareFailedException ex;
		ex.what = std::string("Exception: JointMotorComp::Worker::setSyncPosition::") + s.toStdString();
		throw ex;
	}
}

RoboCompJointMotor::BusParams Worker::getBusParams()
{
	return busParams;
}

void Worker::getMotorParams( const QString & motor , RoboCompJointMotor::MotorParams & mp)
{
	QMutexLocker ml(w_mutex);
	if ( handler->motors.contains( motor ) )
	{
		mp = handler->motors[motor]->params;
	}
	else
	{
		RoboCompJointMotor::UnknownMotorException ex(motor.toStdString());
		throw ex;
	}
}

RoboCompJointMotor::MotorParamsList Worker::getAllMotorParams( )
{
	RoboCompJointMotor::MotorParamsList list;
	QMutexLocker ml(w_mutex);
	
	foreach( Servo *s, handler->motors.values())
	{
		list.push_back( s->params );
	}
	return list;
}

RoboCompJointMotor::MotorStateMap Worker::getAllMotorState( )
{
	RoboCompJointMotor::MotorStateMap map;
	RoboCompJointMotor::MotorState state;
	
	QMutexLocker ml(w_mutex);

	foreach( Servo *s, handler->motors.values())
	{
		state.pos = s->data.currentPosRads;
		state.v = s->data.currentVelocityRads;
		state.p = s->data.currentPos;
		state.isMoving = s->data.isMoving;
		state.temperature = s->data.temperature;
		map[s->params.name] = state ;
	}
		
	return map;
}

void Worker::getState(const QString & motor, RoboCompJointMotor::MotorState & state)
{
	QMutexLocker ml(w_mutex);
	if ( handler->motors.contains( motor ) )
	{
		state.pos = handler->motors[motor]->data.currentPosRads;
		state.v = handler->motors[motor]->data.currentVelocityRads;
		state.p = handler->motors[motor]->data.currentPos;
		state.temperature = handler->motors[motor]->data.temperature;
	}
	else
	{
		RoboCompJointMotor::UnknownMotorException ex(motor.toStdString());
		throw ex;
	}
}
void Worker::setZeroPos(const std::string &motor)
{
	uint i=0;
	while(i<params.size())
	{
		if(params[i].name == motor)
		{
			params[i].zeroPos = handler->motors[QString::fromStdString(motor)]->data.currentPos;
			break;
		}
		i++;
	}
}
void Worker::setSyncZeroPos()
{
	uint j=0;
	for (int i=0; i<busParams.numMotors; i++)
	{
		std::string s = QString::number(i).toStdString();
		j = 0;
		while(j<params.size())
		{
			if(params[j].name == s)
			{
				params[j].zeroPos = handler->motors[QString::fromStdString(s)]->data.currentPos;
				break;
			}
			j++;
		}
	}
}
void Worker::stopAllMotors()
{
	QVector<Dynamixel::GoalPosition> hPositionList;
	float position=0.f, velocity = 0.1;
	QMutexLocker ml(w_mutex);
	
	foreach( Servo *s, handler->motors.values())
	{
		QString motorName = QString::fromStdString( s->params.name);
		if ( handler->motors.contains( motorName )  == false )
		{
			RoboCompJointMotor::UnknownMotorException ex(s->params.name);
			throw ex;
		}
		else
		{
			handler->getPosition(motorName,position);
			Dynamixel::GoalPosition gp(motorName, s->params.busId, position, velocity);
			hPositionList.append( gp );
		}
	}
	try
	{
		handler->setSyncPosition( hPositionList );
	}
	catch( QString &s)
	{
		RoboCompJointMotor::HardwareFailedException ex;
		ex.what = std::string("Exception: DynamixelComp::Worker::setSyncVelocity::") + s.toStdString();
		throw ex;
	}
}
void Worker::stopMotor(const QString &motor)
{
	QMutexLocker ml(w_mutex);
	if ( handler->motors.contains( motor ) )
	{
		float position, velocity = 0.1;
		handler->getPosition(motor,position);
		handler->setPosition(motor,position,velocity);
	}
	else
	{
		RoboCompJointMotor::UnknownMotorException ex(motor.toStdString());
		throw ex;
	}
}
void Worker::releaseBrakeAllMotors()
{
	QMutexLocker ml(w_mutex);
	foreach( Servo *s, handler->motors.values())
	{
		handler->powerOff(QString::fromStdString(s->params.name));
	}
}
void Worker::releaseBrakeMotor(const QString &motor)
{
	QMutexLocker ml(w_mutex);	
	if ( handler->motors.contains( motor ) )
	{
		handler->powerOff(motor);
	}
	else
	{
		RoboCompJointMotor::UnknownMotorException ex(motor.toStdString());
		throw ex;
	}
}

void Worker::enableBrakeAllMotors()
{
	QMutexLocker ml(w_mutex);
	foreach( Servo *s, handler->motors.values())
	{
		handler->powerOn(QString::fromStdString(s->params.name));
	}
}
void Worker::enableBrakeMotor(const QString &motor)
{
	QMutexLocker ml(w_mutex);	
	if ( handler->motors.contains( motor ) )
	{
		handler->powerOn(motor);
	}
	else
	{
		RoboCompJointMotor::UnknownMotorException ex(motor.toStdString());
		throw ex;
	}
}

