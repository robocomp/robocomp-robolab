/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
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
SpecificWorker::SpecificWorker(MapPrx& mprx, QObject *parent) : GenericWorker(mprx, parent)
{
	innerModel = NULL;
}


/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}


void SpecificWorker::compute( )
{
}


bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		innerModel = new InnerModel(par.value);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }


	timer.start(Period);
	return true;
}


void SpecificWorker::newAprilTag(const tagsList &l)
{
	if (l.size()==0 or innerModel==NULL) return;

	static int lastID = -1;

	int indexToUse = 0;
	for (uint i=0; i<l.size(); i++)
	{
		if (l[i].id == lastID)
		{
			indexToUse = i;
		}
	}

	lastID = l[indexToUse].id;
	int i = indexToUse;
	printf("---------------------------------------\n");
	printf("---------------------------------------\n");
	printf("newAprilTag %d\n", (int)l.size());



	//
	// Compute relative position of the robot from the point of view of the tag.
	//
	// T
	const RoboCompAprilTags::tag  &tag = l[i];

	const QString robot_name("robot");
	const QString tagSeenPose("aprilOdometryReference_seen_pose");
	const QString tagReference = QString("aprilOdometryReference%1_pose").arg(tag.id);

	try
	{
		innerModel->updateTransformValues(tagSeenPose, tag.tx, tag.ty, tag.tz, tag.rx, tag.ry, tag.rz);
	}
	catch (...)
	{
		printf("eeee %d\n", __LINE__);
	}


	auto M_T2C = innerModel->getTransformationMatrix("root", tagReference);
	auto M_t2C = innerModel->getTransformationMatrix("root", tagSeenPose);
	auto err = M_T2C * M_t2C.invert();
	auto M_W2R = innerModel->getTransformationMatrix(robot_name, innerModel->getParentIdentifier(robot_name));

	auto correctOdometry = err * M_W2R;

	auto new_T = correctOdometry.getCol(3).fromHomogeneousCoordinates();
	//new_T.print("new T");
	auto new_R = correctOdometry.extractAnglesR_min();
	//new_R.print("new R");


	///
	/// Send corrected odometry to the robot platform
	///
	try
	{
		differentialrobot_proxy->correctOdometer(new_T(0), new_T(2), new_R(1));
	}
	catch( Ice::Exception e)
	{
		fprintf(stderr, "Can't connect to DifferentialRobot\n");
	}
}


