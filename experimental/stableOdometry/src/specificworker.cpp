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
// 	ofstream fs("info.csv", ios_base::out|ios_base::app);
// 	fs<<"id,time,x_base,z_base,alpha_base,x_april,z_april,alpha_april,x_cgr,z_cgr,alpha_cgr,x_error,z_error,alpha_error\n";
// 	fs.close();
	

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
	QMutexLocker l(mutex);
	static QTime lastDistanceUpdate;
	static int lastX, lastZ;
	int x, z;
	float alpha;
	
	if (distance<0)
	{
		distance = 0;
		try
		{
			omnirobot_proxy->getBasePose(lastX, lastZ, alpha);
		}
		catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};
	}

	omnirobot_proxy->getBasePose(x, z, alpha);
	int diffX = lastX - x;
	int diffZ = lastZ - z;
	float inc = sqrt(diffX*diffX + diffZ*diffZ);
// 	printf("distance %f\n", distance);
	distance += inc;
	lastX = x;
	lastZ = z;
}



///////////////////////////////
///SUBSCRIPTION METHODS
///////////////////////////////


void SpecificWorker::newAprilBasedPose(float x, float z, float alpha)
{
//	if (lastAprilUpdate.elapsed() > 1000)
	{ 
		printf("pose: %f %f %f", x, z, alpha);
		try	{	omnirobot_proxy->correctOdometer(x, z, alpha); }
		catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};
		
		try	{	cgr_proxy->resetPose(x, z, alpha); }
		catch(Ice::Exception &ex) {std::cout<<ex.what()<<std::endl;};
		
		lastAprilUpdate = QTime::currentTime();
	}
}


void SpecificWorker::newCGRPose(const float poseCertainty, float x, float z, float alpha)
{
	printf("_\n");
	//if (lastAprilUpdate.elapsed() > 5000)
	{
		//if (lastCGRUpdate.elapsed() > 1000)
		{
//  			printf("%f\n", poseCertainty);
			C = poseCertainty;
//			if (poseCertainty < 0.6)
			{
// 				omnirobot_proxy->correctOdometer(x, z, alpha);
// 				lastCGRUpdate = QTime::currentTime();
// 				RoboCompOmniRobot::TBaseState bState;
// 				try { omnirobot_proxy->getBaseState(bState); }
// 				catch(Ice::Exception &ex) { std::cout<<ex.what()<<std::endl; }

// 				float xC = 0.8 * bState.x + 0.2 * x;
// 				float zC = 0.8 * bState.z + 0.2 * z;
// 				float alphaC = 0.8 * bState.alpha + 0.2 * alpha;
 
	/*			float xC = x;
				float zC = z;
				float alphaC = alpha;
	*/	
				printf("setting  %f  %f  ___  %f\n", x, z, alpha);
				try	{	omnirobot_proxy->correctOdometer(x, z, alpha); }
				catch(Ice::Exception &ex) { std::cout<<ex.what()<<std::endl; }
				lastCGRUpdate = QTime::currentTime();
			}
		}
	}
}



void SpecificWorker::newCGRCorrection(float poseUncertainty, float x1, float z1, float alpha1, float x2, float z2, float alpha2)
{
	static InnerModel *innermodel=NULL;
	static InnerModelTransform *corrSLAMBack;
	static InnerModelTransform *corrSLAMNew;
	static InnerModelTransform *corrREALBack;
	static InnerModelTransform *corrREALNew;

	if (innermodel == NULL)
	{
		innermodel = new InnerModel();
		corrSLAMBack = innermodel->newTransform("corrSLAMBack", "static", innermodel->getRoot(), 0,0,0, 0,0,0, 0);
		innermodel->getRoot()->addChild(corrSLAMBack);
		corrSLAMNew = innermodel->newTransform("corrSLAMNew", "static", innermodel->getRoot(), 0,0,0, 0,0,0, 0);
		innermodel->getRoot()->addChild(corrSLAMNew);

		corrREALBack = innermodel->newTransform("corrREALBack", "static", innermodel->getRoot(), 0,0,0, 0,0,0, 0);
		innermodel->getRoot()->addChild(corrREALBack);
		corrREALNew = innermodel->newTransform("corrREALNew", "static", corrREALBack, 0,0,0, 0,0,0, 0);
		corrREALBack->addChild(corrREALNew);
	}


	innermodel->updateTransformValues("corrSLAMBack",   x1, 0, z1,      0, alpha1, 0  );
	innermodel->updateTransformValues("corrSLAMNew",    x2, 0, z2,      0, alpha2, 0  );
	QVec inc = innermodel->transform6D("corrSLAMBack", "corrSLAMNew");
	
	inc.print("inc");

	RoboCompGenericBase::TBaseState bState;
	try { omnirobot_proxy->getBaseState(bState); }
	catch(Ice::Exception &ex) { std::cout<<ex.what()<<std::endl; }
	innermodel->updateTransformValues("corrREALBack", bState.correctedX,0.,bState.correctedZ,   0,bState.correctedAlpha,0);
	innermodel->updateTransformValues("corrREALNew",   inc(0),0,inc(2),    0,inc(4),0);

	QVec result = innermodel->transform6D("root", "corrREALNew");
	newCGRPose(poseUncertainty, result(0), result(2), result(4));
}

