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
#if COMPILE_GAZEBO==1

#include "gazebohandler.h"
/**
 * \brief Default constructor
 * @param TMechParams Some mechanical params read form config file
*/
GazeboHandler::GazeboHandler(RoboCompDifferentialRobot::TMechParams _params): mechParams(_params)
{
	qDebug() << "GazeboHandler::GazeboHandler()";
	/// Connect to the libgazebo server
printf("a\n");
#ifdef OLD_API
	client = new gazebo::Client();
#else
	client = new libgazebo::Client();
#endif
	try
	{
	  client->ConnectWait(0, GZ_CLIENT_ID_USER_FIRST);
	}
	catch (std::string e)
	{
		qFatal("Gazebo error: Unable to connect client\n");
	}
printf("b\n");
	/// Open the Simulation Interface
#ifdef OLD_API
	simIface = new gazebo::SimulationIface();
#else
	simIface = new libgazebo::SimulationIface();
#endif
	try
	{
		simIface->Open(client, "default");
	}
	catch (std::string e)
	{
		qFatal("Gazebo error: Unable to connect simulation\n");
	}
	/// Open the Position interface
#ifdef OLD_API
	posIface = new gazebo::PositionIface();
#else
	posIface = new libgazebo::PositionIface();
#endif
	try
	{
		posIface->Open(client, mechParams.device.c_str());
	}
	catch (std::string e)
	{
		qFatal("Gazebo error: Unable to connect pose: check device name:wq\n");
	}

	posIface->data->cmdEnableMotors = 1;

	//rotation matrix
	r00=1.f;r01=0.f;r10=0.f;r11=1.f;
	t0=0.f;t1=0.f;
	alpha=0.f;

	rDebug("Starting GazeboHandler...");
}
/**
 * \brief Destructor
 */
GazeboHandler::~GazeboHandler()
{

}

void GazeboHandler::compute()
{

}

/**
 * Initialization variables and connects
 * @param m QMutex for bState data
 * @param mesDim_ deprecated
 */
void GazeboHandler::initMutex(QMutex *m)
{

}
/**
 * Set the base speed in rads/sg
 *
 * @param adv New base speed en mm/sg
 * @param rot New base rot speed en rads/sg
 * @return true if command executed successfully
 */
bool GazeboHandler::setSpeedBase(float adv,float rot)
{
	posIface->Lock(1);
	posIface->data->cmdVelocity.pos.x = adv/(1000.);
	posIface->data->cmdVelocity.yaw = -rot;
	posIface->Unlock();
// 	printf("%f %f\n", adv, rot);
// 	rDebug("set speed");
	return true;
}
/**
 * Stops base
 * @return true if sucess
 */
bool GazeboHandler::stopBase()
{
	posIface->Lock(1);
	posIface->data->cmdVelocity.pos.x = 0;
	posIface->data->cmdVelocity.yaw = 0;
	posIface->Unlock();
	return true;
}

/**
 * Return current base state
 * @return bState
 */

RoboCompGenericBase::TBaseState GazeboHandler::getBaseState()
{
#ifdef OLD_API
    gazebo::Pose pose;
#else
    libgazebo::Pose pose;
#endif

#define ERROR_MAGNIFIER_ANG 0.0000005
#define ERROR_MAGNIFIER_POS 0.0000005

    QString model = QString::fromStdString(mechParams.device).split("::")[0];
    simIface->Lock(1);
    simIface->GetPose2d(model.toStdString().c_str(), pose);
    float realX = -pose.pos.y*1000.;
    float realZ = pose.pos.x*1000.;
    float realYaw = -pose.yaw;
    float readX = -posIface->data->pose.pos.y*1000.;
    float readZ = posIface->data->pose.pos.x*1000.;
    float readYaw = -posIface->data->pose.yaw;
    float incX, incZ, incYaw;
    posIface->Unlock();
    
//     if(readYaw<0)
//       readYaw = 2*M_PI + readYaw;
//     
//     if(realYaw<0)
//       realYaw = 2*M_PI + realYaw;
//     
//     static float readYawAnt = readYaw;
//     static float realYawAnt = realYaw;
//     static float nTurnsRead = 0;
//     static float nTurnsReal = 0;
//     
//     if(readYaw<M_PI/2 && readYawAnt>3*M_PI/2)
//       nTurnsRead+=1.;
//     if(readYaw>3*M_PI/2 && readYawAnt<M_PI/2)
//       nTurnsRead-=1;
//     
//     if(realYaw<M_PI/2 && realYawAnt>3*M_PI/2)
//       nTurnsReal+=1.;
//     if(realYaw>3*M_PI/2 && realYawAnt<M_PI/2)
//       nTurnsReal-=1;
// 
//     readYawAnt = readYaw;
//     realYawAnt = realYaw;
//     
//     float xError = readX-realX;
//     float zError = readZ-realZ;
//     float yawError = readYaw + nTurnsRead*2*M_PI - realYaw -nTurnsReal*2*M_PI;
// 
//     if(fabs(xError)>500.)
//     {
// 	if(xError<0)
// 	  xError=-500;
// 	else
// 	  xError=500;
//     }
//     
//     if(fabs(zError)>500.)
//     {
// 	if(zError<0)
// 	  zError=-500;
// 	else
// 	  zError=500;
//     }


    incYaw = readYaw - antYaw;
    incX = (readX-antX)*cos(antYaw) - (readZ-antZ)*sin(antYaw);
    incZ = (readX-antX)*sin(antYaw) + (readZ-antZ)*cos(antYaw);

    antX=readX;
    antZ=readZ;
    antYaw=readYaw;
    
    float poseX=bState.x, poseZ=bState.z;

    bState.x = poseX + incX*cos(bState.alpha) + incZ*sin(bState.alpha);
    bState.z = poseZ - incX*sin(bState.alpha) + incZ*cos(bState.alpha);
    bState.alpha = bState.alpha + incYaw;
    
/*    bState.x = realX;
    bState.z = realZ;
    bState.alpha = realYaw;*/
    
    return bState;
}


/*RoboCompDifferentialRobot::TBaseState GazeboHandler::getBaseState()
{
// #define TRUE_POSITION
// #ifdef TRUE_POSITION
#ifdef OLD_API
	gazebo::Pose pose;
#else
	libgazebo::Pose pose;
#endif
// 	qDebug() << QString::fromStdString(mechParams.device);
	QString model = QString::fromStdString(mechParams.device).split("::")[0];
	simIface->Lock(1);
	simIface->GetPose2d(model.toStdString().c_str(), pose);
	float x = -pose.pos.y*1000.;
	float z = pose.pos.x*1000.;
	float a = -pose.yaw;
	simIface->Unlock();

// 	posIface->Lock(1);
// 	float x = posIface->data->pose.pos.y*1000.;
// 	float z = posIface->data->pose.pos.x*1000.;
// 	float a = posIface->data->pose.yaw*1000.;
// 	posIface->Unlock();


	bState.x = x;
	bState.z = z;
	bState.alpha = a;
	return bState;
}*/
/**
 * Return current base mechanical params
 * @return TMechParams
 */
RoboCompDifferentialRobot::TMechParams GazeboHandler::getMechParams()
{
	return mechParams;
}

/**
 * Reset odometer
 * @return true if command was sended successfully, else return false
 */
bool GazeboHandler::resetOdometer()
{
        posIface->Lock(1);
	  antX = -posIface->data->pose.pos.y*1000.;
	  antZ = posIface->data->pose.pos.x*1000.;
	  antYaw = -posIface->data->pose.yaw;
	posIface->Unlock();

	bState.x = 0.;
	bState.z = 0.;
	bState.alpha = 0.;

/*	alpha = posIface->data->pose.yaw;
	r00 = sin(alpha);
	r01 = -cos(alpha);
	r10 = cos(alpha);
	r11 = sin(alpha);
	posIface->Lock(1);
	t0 = posIface->data->pose.pos.y*1000.;
	t1 = posIface->data->pose.pos.x*1000.;
	posIface->Unlock();*/
	return true;
}
/**
* Set odometer to a specified state
* @param _bState State to set odometer values
* @return true if command was sended successfully, else return false
*/
bool GazeboHandler::setOdometer(RoboCompGenericBase::TBaseState _bState)
{
        posIface->Lock(1);
	  antX = -posIface->data->pose.pos.y*1000.;
	  antZ = posIface->data->pose.pos.x*1000.;
	  antYaw = -posIface->data->pose.yaw;
	posIface->Unlock();
  
  	bState.x = _bState.x;
	bState.z = _bState.z;
	bState.alpha = _bState.alpha;

/*	posIface->Lock(1);
	posIface->data->pose.pos.x=_bState.z/1000.;
	posIface->data->pose.pos.y=-_bState.x/1000.;
	posIface->data->pose.yaw=-_bState.alpha;
	posIface->data->velocity.pos.x=0.;
	posIface->data->velocity.yaw=0.;
	posIface->Unlock();*/
	return true;
}

void GazeboHandler::readMechParams()
{


}

#endif
