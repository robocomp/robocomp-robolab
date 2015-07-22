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

SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	lastAprilUpdate = QTime::currentTime().addSecs(-1000);
	lastCGRUpdate   = QTime::currentTime().addSecs(-1000); 
	ofstream fs("info.csv", ios_base::out|ios_base::app);
	fs<<"id,time,x_base,z_base,alpha_base,x_april,z_april,alpha_april,x_cgr,z_cgr,alpha_cgr,x_error,z_error,alpha_error\n";
	fs.close();
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
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

	RoboCompOmniRobot::TBaseState bState;
	omnirobot_proxy->getBaseState(bState);
	ofstream fs("info.csv", ios_base::out|ios_base::app);
	fs << id << ","<<lastAprilUpdate.elapsed()
	<<","<< bState.x <<","<< bState.z << ","<< bState.alpha <<","<< x <<","<< z << ","<< alpha 
	<<","<< bState.correctedX <<","<< bState.correctedZ << ","<< bState.correctedAlpha 
	<<","<< fabs(bState.correctedX-x) <<","<< fabs(bState.correctedZ-z) << ","<< fabs(bState.correctedAlpha-alpha)<<"\n";
	fs.close();
	id++;
	lastAprilUpdate = QTime::currentTime();
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
	//if (lastAprilUpdate.elapsed() > 1000 + 2000)
	//{
	//	if (lastCGRUpdate.elapsed() > 1000)
		{
			printf("%f\n", poseCertainty);
	//		if (poseCertainty > 0.4)
			{
				omnirobot_proxy->correctOdometer(x, z, alpha);
				lastCGRUpdate = QTime::currentTime();
			}
		}
	//}
}
