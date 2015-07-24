/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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
ofstream fs("info.csv");

SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	lastAprilUpdate = QTime::currentTime().addSecs(-1000);
	lastCGRUpdate   = QTime::currentTime().addSecs(-1000);

	distance = -1;
	fs<<"id,dist,C,time,x_base,z_base,alpha_base,x_april,z_april,alpha_april,x_cgr,z_cgr,alpha_cgr\n";

	timer.start(2000);

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	fs.close();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
	QMutexLocker l(mutex);
	static QTime lastDistanceUpdate;
	static int lastX, lastZ;
	int x, z;
	float alpha;
	
	
	if (distance<0)
	{
		distance = 0;
		omnirobot_proxy->getBasePose(lastX, lastZ, alpha);
	}

	omnirobot_proxy->getBasePose(x, z, alpha);
	int diffX = lastX - x;
	int diffZ = lastZ - z;
	float inc = sqrt(diffX*diffX + diffZ*diffZ);
	printf("distance %f\n", distance);
	distance += inc;
	lastX = x;
	lastZ = z;
}



///////////////////////////////
///SUSCRIBES METHODS
///////////////////////////////
/*
void SpecificWorker::newAprilBasedPose(float x, float z, float alpha)
{
	if (lastAprilUpdate.elapsed() > 1000)
	{
		printf("pose: %f %f %f", x, z, alpha);
		omnirobot_proxy->correctOdometer(x, z, alpha);
		cgr_proxy->resetPose(x, z, alpha);
		lastAprilUpdate = QTime::currentTime();
	}
}
*/
#ifdef DEBUG
void SpecificWorker::newAprilBasedPose(float x, float z, float alpha)
{	
	static int id=0;
	static bool relocation = true;
	if (lastAprilUpdate.elapsed() > 1000)
	{
		if (relocation)
		{
			printf("pose: %f %f %f", x, z, alpha);
			omnirobot_proxy->correctOdometer(x, z, alpha);
			cgr_proxy->resetPose(x, z, alpha);
			lastAprilUpdate = QTime::currentTime();
			relocation = false;
		}	
		else 
		{
			QMutexLocker l(mutex);
			RoboCompOmniRobot::TBaseState bState;
			omnirobot_proxy->getBaseState(bState);
			fs << id << ","
			<< distance << ","
			<< C << ","
			<< lastAprilUpdate.elapsed() <<","
			<< bState.x <<","<< bState.z << ","<< bState.alpha <<","
			<< x <<","<< z << ","<< alpha <<","
			<< bState.correctedX <<","<< bState.correctedZ << ","<< bState.correctedAlpha << "\n";
			fs.flush();
			
			float xErr = bState.correctedX - x;
			float zErr = bState.correctedZ - z;
			float err = sqrt(xErr*xErr + zErr*zErr);
			cout << distance << " " << err << "\n";
			id++;
			lastAprilUpdate = QTime::currentTime();
		}
	}
}

#else 
void SpecificWorker::newAprilBasedPose(float x, float z, float alpha)
{
	if (lastAprilUpdate.elapsed() > 1000)
	{ 
		printf("pose: %f %f %f", x, z, alpha);
		omnirobot_proxy->correctOdometer(x, z, alpha);
		cgr_proxy->resetPose(x, z, alpha);
		lastAprilUpdate = QTime::currentTime();
	}
}

#endif

void SpecificWorker::newCGRPose(const float poseCertainty, float x, float z, float alpha)
{
	if (lastAprilUpdate.elapsed() > 1000 + 2000)
	{
		if (lastCGRUpdate.elapsed() > 2000)
		{
			printf("%f\n", poseCertainty);
			C = poseCertainty;
			if (poseCertainty < 0.6)
			{
				omnirobot_proxy->correctOdometer(x, z, alpha);
				lastCGRUpdate = QTime::currentTime();
			}
		}
	}
}
