/*
 *    Copyright (C) 2006-2011 by RoboLab - University of Extremadura
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

Worker::Worker(RoboCompOmniRobot::OmniRobotPrx omnirobotprx, RoboCompJointMotor::JointMotorPrx jointmotorprx, RoboCompLaser::LaserPrx laserprx, WorkerConfig &cfg) : QObject()
{
	omnirobot = omnirobotprx;
	jointmotor = jointmotorprx;
	laser = laserprx;

	rgbds          = cfg.rgbdsVec;
	virtualLaserID = QString::fromStdString(cfg.laserBaseID);
	actualLaserID  = QString::fromStdString(cfg.actualLaserID);
	minHeight      = cfg.minHeight;
	minHeightNeg   = cfg.minHeightNeg;
	maxHeight      = cfg.maxHeight;
	LASER_SIZE     = cfg.LASER_SIZE;
	MIN_LENGTH     = cfg.MIN_LENGTH;
	maxLength      = cfg.maxLength;
	updateJoint    = cfg.updateJoint;
	DECIMATION_LEVEL = cfg.DECIMATION_LEVEL;


	mutex = new QMutex(QMutex::Recursive);

	innerModel = new InnerModel(cfg.xmlpath);

	connect(&timer, SIGNAL(timeout()), this, SLOT(compute()));

	/// Resize and initialization
	laserDataR = new RoboCompLaser::TLaserData;
	laserDataW = new RoboCompLaser::TLaserData;
	laserDataR->resize(LASER_SIZE);
	laserDataW->resize(LASER_SIZE);
	for (int32_t i=0; i<LASER_SIZE; i++)
	{
		(*laserDataR)[i].angle = (double(i)-(0.5*LASER_SIZE))*((2.*M_PIl)/LASER_SIZE);
		(*laserDataW)[i].angle = (double(i)-(0.5*LASER_SIZE))*((2.*M_PIl)/LASER_SIZE);
	}
	printf("Direct field of view < %f -- %f >\n", (*laserDataR)[0].angle, (*laserDataR)[laserDataR->size()-1].angle);

	RoboCompOmniRobot::TBaseState oState;
	try { omnirobot->getBaseState(oState); }
	catch (Ice::Exception e) { qDebug() << "error talking to base" << e.what(); }
	innerModel->updateTransformValues("robot", bStateOut.x, 0, bStateOut.z, 0, bStateOut.alpha,0);
	bState.x     = oState.x;
	bState.z     = oState.z;
	bState.alpha = oState.alpha;
innerModel->transform("root", "root").print("robot");

	printf("<<< virtualLaserID: %s>>>\n", virtualLaserID.toStdString().c_str());

	map = new LMap(2.2*maxLength, 250, maxLength, "movableRoot", virtualLaserID, innerModel, "robot", 290);

	confData.staticConf = 1;
	confData.maxMeasures = 100;
	confData.iniRange = -3;
	confData.endRange = 3;

	confData.maxDegrees = 6;
	confData.maxRange = maxLength;
	confData.minRange = MIN_LENGTH;
	confData.angleRes = 2.*M_PIl/LASER_SIZE;//confData.maxDegrees/confData.maxMeasures;
	confData.driver = "simulated from depth map";
	confData.device = "rgbd";

	compute();
	timer.start(100);
}

/**
* \brief Default destructor
*/
Worker::~Worker()
{

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
	Period = p*1000;
}
/**
* \brief
* @param params_ Parameter list received from monitor thread
*/
bool Worker::setParams(RoboCompCommonBehavior::ParameterList params_)
{
	return true;
}


int32_t Worker::angle2bin(double ang)
{
	QMutexLocker m(mutex);
	while (ang>M_PI)  ang -= 2.*M_PI;
	while (ang<-M_PI) ang += 2.*M_PI;

	double ret;
	ang += localFOV/2;
	ret = (ang * LASER_SIZE) / localFOV;
	return int32_t(ret);
}

/**
* \brief Thread method
*/
void Worker::updateInnerModel()
{
	QMutexLocker m(mutex);
	/// Update InnerModel with joint information
	if (updateJoint)
	{
		RoboCompJointMotor::MotorStateMap motorMap;
		try
		{
			jointmotor->getAllMotorState(motorMap);
			for (RoboCompJointMotor::MotorStateMap::iterator it=motorMap.begin(); it!=motorMap.end(); ++it)
			{
				innerModel->updateJointValue(it->first.c_str(), it->second.pos);
			}
		}
		catch (const Ice::Exception &ex)
		{
			cout << "Can't connect to jointMotor: " << ex << endl;
		}
	}
	else
	{
		printf("not using joint\n");
	}


	RoboCompOmniRobot::TBaseState oState;
	try { omnirobot->getBaseState(oState); }
	catch (Ice::Exception e) { qDebug()<<"error talking to base"<<e.what(); }
	bStateOut.x     = oState.x;
	bStateOut.z     = oState.z;
	bStateOut.alpha = oState.alpha;
	innerModel->updateTransformValues("robot", bStateOut.x, 0, bStateOut.z, 0, bStateOut.alpha, 0);
}


void Worker::compute()
{

	/// Update InnerModel
	updateInnerModel();

	/// Clear laser measurement
	for (int32_t i=0; i<LASER_SIZE; ++i)
	{
		(*laserDataW)[i].dist = maxLength;
	}

	map->update_timeAndPositionIssues(actualLaserID);

	// Include laser data
	try
	{
		RoboCompLaser::TLaserData alData = laser->getLaserData();
		map->update_include_laser(&alData, actualLaserID);
	}
	catch (const Ice::Exception &ex)
	{
		cout << "Can't connect to laser: " << ex << endl;
		exit(-1);
	}

	/// Include each of the RGBD proxies
	for (uint r=0; r<rgbds.size(); ++r)
	{
		if (rgbds[r].bus != true) /// If the proxy is a bus
		{
			//printf("<< %s\n", rgbds[r].id.c_str());
			RoboCompRGBD::PointSeq points;
			try
			{
				rgbds[r].proxyRGBD->getXYZ(points, hState, bState);
			}
			catch (const Ice::Exception &ex)
			{
				cout << "Can't connect to rgbd: " << ex << endl;
				continue;
			}
			//printf("   %s >>\n", rgbds[r].id.c_str());
			printf("<\n");
			map->update_include_rgbd(&points, QString::fromStdString(rgbds[r].id));
			printf(">\n");
		}
	}

	map->update_done(actualLaserID, MIN_LENGTH);



	// Double buffer swap
	RoboCompLaser::TLaserData *t;
	map->getLaserData(laserDataW, LASER_SIZE, maxLength);
	mutex->lock();
	t = laserDataR;
	laserDataR = laserDataW;
	laserDataW = t;
	mutex->unlock();

}



RoboCompLaser::TLaserData Worker::getLaserData()
{
	QMutexLocker m(mutex);
	return *laserDataR;
}

RoboCompLaser::TLaserData Worker::getLaserAndBStateData(RoboCompDifferentialRobot::TBaseState &baseState)
{
	QMutexLocker m(mutex);
	baseState = bStateOut;
	return *laserDataR;
}

RoboCompLaser::LaserConfData Worker::getLaserConfData()
{
	QMutexLocker m(mutex);
	return confData;
}

