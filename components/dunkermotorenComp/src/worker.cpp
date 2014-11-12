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

Worker::Worker(QObject *parent) : QThread(parent)
{
 	frequency = 50;
	active = false;
	handler = NULL;
	w_mutex = new QMutex(QMutex::Recursive);
	monitor_mutex = new QMutex(QMutex::Recursive);
	qDebug() << "JointMotor::Worker::Worker() - constructor Ok";
}

Worker::~Worker()
{
	if (handler) delete handler;
	if (w_mutex) delete w_mutex;
	if (monitor_mutex) delete monitor_mutex;
}

void Worker::run( )
{
	std::cout << "dunkermotoren::run() Starting update loop." << std::endl;
	for (;;)
	{
		if (active)
		{
			handler->update();
			usleep(busParams.basicPeriod*1000);
		}
	}
}

void Worker::setFrequency(int freq)
{
	qDebug()<<"change frequency";
	QMutexLocker lock(monitor_mutex);
	busParams.basicPeriod = freq;
}

int Worker::getFrequency()
{
	QMutexLocker lock(monitor_mutex);
	return frequency;
}

void Worker::setParams(RoboCompCommonBehavior::ParameterList _params)
{
	QMutexLocker lock(monitor_mutex);
	active = false;

	qDebug() << "dunkermotorenComp::Worker::setParams()";

	if (_params["dunkermotoren.Device"].value != busParams.device)
	{
		if(handler!= NULL) delete handler;
		busParams.device = _params["dunkermotoren.Device"].value;
		busParams.handler = _params["dunkermotoren.Handler"].value;
		busParams.numMotors = QString::fromStdString(_params["dunkermotoren.NumMotors"].value).toInt();
		busParams.baudRate = QString::fromStdString(_params["dunkermotoren.BaudRate"].value).toInt();
		busParams.basicPeriod = QString::fromStdString(_params["dunkermotoren.BasicPeriod"].value).toInt();

		for (int i=0; i<busParams.numMotors; i++)
		{
			std::string s= QString::number(i).toStdString();
			RoboCompJointMotor::MotorParams mpar;
			DunkerParams dpar;
			mpar.name = _params["dunkermotoren.Params_" + s +".name"].value;
			mpar.busId = QString::fromStdString(_params["dunkermotoren.Params_" + s +".busId"].value).toUShort();
			mpar.invertedSign = QString::fromStdString(_params["dunkermotoren.Params_" + s +".invertedSign"].value).contains("true");
			mpar.minPos = QString::fromStdString(_params["dunkermotoren.Params_" + s +".minPos"].value).toFloat();
			mpar.maxPos = QString::fromStdString(_params["dunkermotoren.Params_" + s +".maxPos"].value).toFloat();
			mpar.zeroPos = QString::fromStdString(_params["dunkermotoren.Params_" + s +".zeroPos"].value).toFloat();
			mpar.maxVelocity = QString::fromStdString(_params["dunkermotoren.Params_" + s +".maxVelocity"].value).toFloat();
			mpar.stepsRange = QString::fromStdString(_params["dunkermotoren.Params_" + s +".stepsToRadsRatio"].value).toFloat();
			
			dpar.maxPosErr = QString::fromStdString(_params["dunkermotoren.Params_" + s +".maxPosErr"].value).toInt();
			dpar.accV = QString::fromStdString(_params["dunkermotoren.Params_" + s +".accV"].value).toInt();
			dpar.accT = QString::fromStdString(_params["dunkermotoren.Params_" + s +".accT"].value).toInt();
			dpar.decV = QString::fromStdString(_params["dunkermotoren.Params_" + s +".decV"].value).toInt();
			dpar.decT = QString::fromStdString(_params["dunkermotoren.Params_" + s +".decT"].value).toInt();
			dpar.gearRev = QString::fromStdString(_params["dunkermotoren.Params_" + s +".gearRev"].value).toFloat();
			dpar.motorRev = QString::fromStdString(_params["dunkermotoren.Params_" + s +".motorRev"].value).toFloat();
			dpar.posCurLim = QString::fromStdString(_params["dunkermotoren.Params_" + s +".posCurLim"].value).toInt();
			dpar.negCurLim = QString::fromStdString(_params["dunkermotoren.Params_" + s +".negCurLim"].value).toInt();
			dpar.curPeak = QString::fromStdString(_params["dunkermotoren.Params_" + s +".curPeak"].value).toInt();
			dpar.curContin = QString::fromStdString(_params["dunkermotoren.Params_" + s +".curContin"].value).toInt();
			dpar.curTime = QString::fromStdString(_params["dunkermotoren.Params_" + s +".curTime"].value).toInt();
			dpar.encoderRes = QString::fromStdString(_params["dunkermotoren.Params_" + s +".encodeRes"].value).toInt();
			dpar.setPID =  QString::fromStdString(_params["dunkermotoren.Params_" + s +".setPID"].value).contains("true");
			dpar.velKp =  QString::fromStdString(_params["dunkermotoren.Params_" + s +".velKp"].value).toInt();
			dpar.velKi =  QString::fromStdString(_params["dunkermotoren.Params_" + s +".velKi"].value).toInt();
			dpar.velKd =  QString::fromStdString(_params["dunkermotoren.Params_" + s +".velKd"].value).toInt();

			params.push_back(mpar);
			dunkerParams[mpar.busId] = dpar;
		}

		handler = new Dunkermotoren(&busParams, &params, &dunkerParams, w_mutex);
		qDebug() << "dunkermotoren::Worker::setParams() Dunkermotoren handler";

		try
		{
			handler->initialize();
		}
		catch(QString &s)
		{
			qDebug()<<"ERROR trying to initialize.";
		}
		this->start();
	}
	active = true;
}


///
/// Servant methods
///

void Worker::setPosition(const RoboCompJointMotor::MotorGoalPosition & goalPosition)
{
	try
	{
		handler->setPosition(QString::fromStdString( goalPosition.name) , goalPosition.position, goalPosition.maxSpeed);
	}
	catch (const MotorHandlerErrorWritingToPortException &ex)
	{
		qDebug()<<"ERROR setting position";
		RoboCompJointMotor::HardwareFailedException ex2;
		ex2.what = std::string("Exception: JointMotorComp::Worker::setPosition::HardwareFailedException ") + ex.what();
	}
	catch (const MotorHandlerUnknownMotorException &ex)
	{
		qDebug()<<"ERROR setting position";
		RoboCompJointMotor::UnknownMotorException ex2;
		ex2.what = std::string("Exception: JointMotorComp::Worker::setPosition::UnknownMotorException ") + ex.what();
	}
}

void Worker::setVelocity(const RoboCompJointMotor::MotorGoalVelocity & goalVelocity)
{
	try
	{
		printf("Worker::setVelocity() --> %f\n", goalVelocity.velocity);
		handler->setVelocity(QString::fromStdString( goalVelocity.name) , goalVelocity.velocity);
	}
	catch( const MotorHandlerErrorWritingToPortException &ex )
	{
		qDebug()<<"ERROR setting velocity";
		RoboCompJointMotor::HardwareFailedException ex2;
		ex2.what = std::string("Exception: JointMotorComp::Worker::setVelocity::HardwareFailedException ") + ex.what();
	}
	catch( const MotorHandlerUnknownMotorException &ex )
	{
		qDebug()<<"ERROR setting velocity";
		RoboCompJointMotor::UnknownMotorException ex2;
		ex2.what= std::string("Exception: JointMotorComp::Worker::setVelocity::UnknownMotorException") + ex.what();
	}
}


void Worker::setReferenceVelocity( const RoboCompJointMotor::MotorGoalVelocity & goalVelocity )
{
	printf("setRefVel\n");
	try
	{
		QMutexLocker lo(w_mutex);
		const QString motorName = QString::fromStdString(goalVelocity.name);
		int32_t rpms = goalVelocity.velocity * (60./(2.*M_PI)) * 72.;
		RoboCompJointMotor::MotorParams mp;
		getMotorParams(motorName, mp);
		if (mp.invertedSign)
		{
			rpms = -rpms;
		}
		if (handler->setReferenceVelocity(motorName, rpms) == false)
		{
			qDebug()<<"ERROR setting reference velocity.";
			RoboCompJointMotor::HardwareFailedException ex(goalVelocity.name);
		}
	}
	catch(...)
	{
		qDebug()<<"ERROR setting reference velocity. Unknown motor: "<<goalVelocity.name.c_str();
		RoboCompJointMotor::UnknownMotorException ex(goalVelocity.name);
	}
}


void Worker::setSyncPosition( const RoboCompJointMotor::MotorGoalPositionList & goalPosList)
{
	QVector<Dunkermotoren::GoalPosition> hPositionList;
	w_mutex->lock();
	for(uint i=0; i< goalPosList.size() ; i++)
	{
		QString motorName = QString::fromStdString( goalPosList[i].name);
		if ( handler->motors.contains( motorName )  == false )
		{
			qDebug()<<"ERROR setting sync position. Unknown motor: "<<motorName;
			RoboCompJointMotor::UnknownMotorException ex(goalPosList[i].name);
			w_mutex->unlock();
			return;
		}
		else
		{
			Servo *servo = handler->motors[motorName];
			Dunkermotoren::GoalPosition gp(motorName, servo->params.busId, goalPosList[i].position, goalPosList[i].maxSpeed);
			hPositionList.append( gp );
		}
	}
	try
	{
		handler->setSyncPosition( hPositionList );
	}
	catch( QString &s)
	{
		qDebug()<<"ERROR setting sync position.";
		RoboCompJointMotor::HardwareFailedException ex;
		ex.what = std::string("Exception: JointMotorComp::Worker::setSyncPosition::") + s.toStdString();
		w_mutex->unlock();
	}
	w_mutex->unlock();
}

void Worker::setSyncVelocity( const RoboCompJointMotor::MotorGoalVelocityList & goalVelList)
{
	QVector<Dunkermotoren::GoalVelocity> VelocityList;
	w_mutex->lock();
	for (uint i=0; i< goalVelList.size() ; i++)
	{
		QString motorName = QString::fromStdString( goalVelList[i].name);
		if ( handler->motors.contains( motorName )  == false )
		{
			qDebug()<<"ERROR sending command for not existing motor: "<<motorName;
			RoboCompJointMotor::UnknownMotorException ex(goalVelList[i].name);
			w_mutex->unlock();
			return;
		}
		else
		{
			Servo *servo = handler->motors[motorName];
			Dunkermotoren::GoalVelocity gp(motorName, servo->params.busId , goalVelList[i].velocity, goalVelList[i].maxAcc);
			VelocityList.append( gp );
		}
	}
	try
	{
		handler->setSyncReferenceVelocity( VelocityList );
	}
	catch( QString &s)
	{
		qDebug()<<"Error calling setSyncReferenceVelocity";
		RoboCompJointMotor::HardwareFailedException ex;
		ex.what = std::string("Exception: JointMotorComp::Worker::setSyncPosition::") + s.toStdString();
		w_mutex->unlock();
	}
	w_mutex->unlock();
	
}

RoboCompJointMotor::BusParams Worker::getBusParams()
{
	return busParams;
}

void Worker::getMotorParams( const QString & motor , RoboCompJointMotor::MotorParams & mp)
{
	w_mutex->lock();
	if ( handler->motors.contains( motor ) )
	{
		mp = handler->motors[motor]->params;
	}
	else
	{
		qDebug()<<"ERROR trying to get params from unknown motor:"<<motor;
		RoboCompJointMotor::UnknownMotorException ex(motor.toStdString());
		w_mutex->unlock();
	}
	w_mutex->unlock();
}

RoboCompJointMotor::MotorParamsList Worker::getAllMotorParams( )
{
	RoboCompJointMotor::MotorParamsList list;
	w_mutex->lock();
	foreach( Servo *s, handler->motors)
	{
		list.push_back( s->params );
	}
	w_mutex->unlock();
	return list;
}

RoboCompJointMotor::MotorStateMap Worker::getAllMotorState( )
{
// 	qDebug()<<"worker::getAllMotorState()";
	RoboCompJointMotor::MotorStateMap map;
	RoboCompJointMotor::MotorState state;
	w_mutex->lock();

	foreach( Servo *s, handler->motors)
	{
		state.pos = s->data.currentPosRads;
		state.v = s->data.currentVelocityRads;
		state.p = s->data.currentPos;
		state.isMoving = s->data.isMoving;
		map[s->params.name] = state ;
	}
	w_mutex->unlock();
	return map;
}

void Worker::getState(const QString & motor, RoboCompJointMotor::MotorState & state)
{
	w_mutex->lock();
	if (handler->motors.contains(motor))
	{
		state.pos = handler->motors[motor]->data.currentPosRads;
		state.v = handler->motors[motor]->data.currentVelocityRads;
		state.p = handler->motors[motor]->data.currentPos;
	}
	else
	{
		qDebug() << "ERROR trying to get state from unknown motor: " << motor;
		RoboCompJointMotor::UnknownMotorException ex(motor.toStdString());
		w_mutex->unlock();
		throw ex;
		
	}
	w_mutex->unlock();
}

void Worker::setZeroPos(const std::string &name)
{
	w_mutex->lock();
	handler->setZeroPos(QString::fromStdString(name));
	w_mutex->unlock();
}

void Worker::setSyncZeroPos()
{
	w_mutex->lock();
	handler->setSyncZeroPos();
	w_mutex->unlock();
}


