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
	// Initial checks
	if (l.size()==0 or innerModel==NULL) return;
	// Tag selection
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
	const RoboCompAprilTags::tag  &tag = l[indexToUse];

	// Localization-related node identifiers
	const QString robot_name("robot");
	const QString tagSeenPose("aprilOdometryReference_seen_pose");
	const QString tagReference = QString("aprilOdometryReference%1_pose").arg(tag.id);

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
	

	try { aprilbasedlocalization_proxy->newPose(new_T(0), new_T(2), new_R(1)); }
	catch( Ice::Exception e) { fprintf(stderr, "Can't connect to aprilbasedlocalization proxy\n"); }
	
	printf("%f %f   @ %f\n", new_T(0), new_T(2), new_R(1));
}


