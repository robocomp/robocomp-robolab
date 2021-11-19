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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
    if(handler != NULL)
        delete handler;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

    handler = new Dynamixel(params, w_mutex);

	this->Period = period;
	if(this->startup_check_flag)
		this->startup_check();
	else
		timer.start(Period);
}

void SpecificWorker::compute()
{
    handler->update();
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

//////////////////////////////////////////
/// Servant methods
///////////////////////////////////////////
RoboCompJointMotorSimple::MotorParams SpecificWorker::JointMotorSimple_getMotorParams(std::string motor)
{


}

RoboCompJointMotorSimple::MotorState SpecificWorker::JointMotorSimple_getMotorState(std::string motor)
{


}

void SpecificWorker::JointMotorSimple_setPosition(std::string name, RoboCompJointMotorSimple::MotorGoalPosition goal)
{
    QMutexLocker ml(w_mutex);
    try
    {
        handler->setPosition( QString::fromStdString(name), goal.position, goal.maxSpeed);
    }
    catch( const MotorHandlerErrorWritingToPortException &ex )
    {
        RoboCompJointMotorSimple::HardwareFailedException hFailed;
        hFailed.what = std::string("Exception: JointMotorComp::Worker::setPosition::") + ex.what();
        throw hFailed;
    }
    catch( const MotorHandlerUnknownMotorException &ex )
    {
        RoboCompJointMotorSimple::HardwareFailedException hFailed;
        hFailed.what = std::string("Exception: JointMotorComp::Worker::setPosition::") + ex.what();
        throw hFailed;
    }
}

void SpecificWorker::JointMotorSimple_setVelocity(std::string name, RoboCompJointMotorSimple::MotorGoalVelocity goal)
{


}

void SpecificWorker::JointMotorSimple_setZeroPos(std::string name)
{
//implementCODE

}

/**************************************/
// From the RoboCompJointMotorSimple you can use this types:
// RoboCompJointMotorSimple::MotorState
// RoboCompJointMotorSimple::MotorParams
// RoboCompJointMotorSimple::MotorGoalPosition
// RoboCompJointMotorSimple::MotorGoalVelocity

