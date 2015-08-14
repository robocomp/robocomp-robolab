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
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	innerModel = NULL;
	robot_name = QString("robot");
	tagSeenPose = QString("aprilOdometryReference_seen_pose");

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
printf("<<<\n");
	try
	{
		printf("%d\n", __LINE__);
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelCameraName");
		printf("%d\n", __LINE__);
		cameraName = QString::fromStdString(par.value);
		printf("%d\n", __LINE__);
	}
	catch(std::exception e) { qFatal("Error reading config params \"InnerModelCameraName\""); }


	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		innerModel = new InnerModel(par.value);
	}
	catch(std::exception e) { qFatal("Error reading config params \"InnerModel\""); }

	try
	{
		printf("%d\n", __LINE__);
		if (innerModel->getNode("aprilOdometryReference_seen_pose") == NULL)
		{
			printf("%d %s\n", __LINE__, cameraName.toStdString().c_str());
			InnerModelNode *parent = innerModel->getNode(cameraName);
			if (not parent)
				qFatal("can't find im node %s", cameraName.toStdString().c_str());
			cout << parent << endl;
			printf("%d\n", __LINE__);
			cout << parent << endl;
			parent = parent->parent;
			cout << parent << endl;
			printf("%d\n", __LINE__);
			cout << parent << endl;
			InnerModelTransform *tr;
			try
			{
				printf("%d\n", __LINE__);
				tr = innerModel->newTransform("aprilOdometryReference_seen_pose", "static", parent, 0,0,0, 0,0,0);
				printf("%d\n", __LINE__);
				parent->addChild(tr);
				printf("%d\n", __LINE__);
			}
			catch (QString err)
			{
				printf("%s:%s:%d: Exception: %s\n", __FILE__, __FUNCTION__, __LINE__, err.toStdString().c_str());
				throw;
			}
		}
		else
		{
			 qFatal("Error including aprilOdometryReference_seen_pose in InnerModel");
		}
	}
	catch(std::exception e) { qFatal("Error including aprilOdometryReference_seen_pose in InnerModel"); }

printf(">>>\n");

	timer.start(Period);
	return true;
}


void SpecificWorker::newAprilTag(const tagsList &l)
{
	// Initial checks
	if (l.size()==0 or innerModel==NULL)
		return;

	// Tag selection
	static int lastID = -1;
	int indexToUse = -1;
	for (uint i=0; i<l.size(); i++)
	{
		const QString tagReference = QString("aprilOdometryReference%1_pose").arg(l[i].id);
		if (l[i].id == lastID   or   (indexToUse==-1 and innerModel->getNode(tagSeenPose))   )
		{
			indexToUse = i;
		}
	}
	if (indexToUse == -1)
		return;


	lastID = l[indexToUse].id;
	const RoboCompAprilTags::tag  &tag = l[indexToUse];

	// Localization-related node identifiers
	const QString tagReference = QString("aprilOdometryReference%1_pose").arg(tag.id);

	
	printf("\n\n-------------------------------------------------------------------------\n");
	printf("ID: %d\n", tag.id);
	// Update seen tag in InnerModel
	try { innerModel->updateTransformValues(tagSeenPose, tag.tx, tag.ty, tag.tz, tag.rx, tag.ry, tag.rz); }
	catch (...) { printf("Can't update node %s (%s:%d)\n", tagSeenPose.toStdString().c_str(), __FILE__, __LINE__); }

	// Compute the two ways to get to the tag, the difference-based error and the corrected odometry matrix
	const auto M_T2C = innerModel->getTransformationMatrix("root", tagReference);
	const auto M_t2C = innerModel->getTransformationMatrix("root", tagSeenPose);
	const auto err = M_T2C * M_t2C.invert();
	const auto M_W2R = innerModel->getTransformationMatrix(robot_name, innerModel->getParentIdentifier(robot_name));
	const auto correctOdometry = err * M_W2R;

	// Extract the scalar values from the pose matrix and send the correction
	const auto new_T = correctOdometry.getCol(3).fromHomogeneousCoordinates();
	const auto new_R = correctOdometry.extractAnglesR_min();

	new_R.print("R");

	printf("%f %f   @ %f\n", new_T(0), new_T(2), new_R(1));
	aprilbasedlocalization_proxy->newAprilBasedPose(new_T(0), new_T(2), new_R(1));
}
