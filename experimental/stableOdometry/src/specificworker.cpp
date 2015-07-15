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
	finalPose.x=0;
	finalPose.z=0;
	finalPose.alpha=0;
	resetCGR = false;
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

	mutex.lock();
	omnirobot_proxy->correctOdometer(finalPose.x,finalPose.z,finalPose.alpha);
	mutex.unlock();
}



///////////////////////////////
///SUSCRIBES METHODS
///////////////////////////////

void SpecificWorker::newAprilBasedPose(float x, float z, float alpha)
{
	mutex.lock();
	finalPose.x=x;
	finalPose.z=z;
	finalPose.alpha=alpha;
	resetCGR=false;
	mutex.unlock();
  	cgr_proxy->resetPose(finalPose.x,finalPose.z,finalPose.alpha);
}


void SpecificWorker::newCGRPose(const float poseUncertainty, float x, float z, float alpha)
{
	mutex.lock();
	printf("incertidumbre %f\n",poseUncertainty);
	if(poseUncertainty > 0.4 && !resetCGR)
	{
		finalPose.x=x;
		finalPose.z=z;
		finalPose.alpha=alpha;
	}
	else 
		resetCGR=true;
	mutex.unlock();
}

