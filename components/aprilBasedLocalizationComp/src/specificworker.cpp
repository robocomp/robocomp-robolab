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
		
		auto imNode = innerModel->getNode("world");
		InnerModelTransform *tr = innerModel->newTransform("marcaVista", "static", imNode, 0,0,0, 0,0,0);
		imNode->addChild(tr);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }
	

	timer.start(Period);
	return true;
}


void SpecificWorker::newAprilTag(const tagsList &l)
{
	if (l.size()==0) return;

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


	//CAMBIO PARA QUE SE QUEDE CON UNA UNICA MARCA
	//for (uint i=0; i<l.size(); i++)
	//{
		//
		// Compute relative position of the robot from the point of view of the tag.
		//
		// T
		const RoboCompAprilTags::tag  &tag = l[i];
		
		try
		{
			innerModel->updateTransformValues("marcaVista", tag.tx, tag.ty, tag.tz, tag.rx, tag.ry, tag.rz);
// 			innerModel->transform("world", QVec::vec6(tag.tx, tag.ty, tag.tz, tag.rx, tag.ry, tag.rz), "poseRGBCeling");
		}
		catch (...)
		{
			printf("eeee %d\n", __LINE__);
		}
		
		QVec coordInItem = QVec::vec3(0,0,0);
		QVec robot_from_tag = innerModel->transform("marca", "robot");
		/// R
		QMat base_to_marca_rotation = innerModel->getTransformationMatrix("base", "marca");

		float newAngleMarca = base_to_marca_rotation.extractAnglesR_min()(1); //M_PIl
		while (newAngleMarca>+M_PIl) newAngleMarca -= 2.*M_PIl;
		while (newAngleMarca<-M_PIl) newAngleMarca += 2.*M_PIl;
		qDebug()<<newAngleMarca;
		qDebug()<<newAngleMarca;
		float robot_angle_from_tag = newAngleMarca; 
		printf("(%f, %f, %f) angle: %f\n", robot_from_tag(0), robot_from_tag(1), robot_from_tag(2), robot_angle_from_tag);

		//
		// Select reference tag based on its identifier
		//
		std::ostringstream stringStream;
		stringStream << "ceilingReference" << tag.id;
		QString referencia = QString::fromStdString(stringStream.str());

		///
		/// Compute new T
		///
		QVec robot_from_world = innerModel->transform("world", robot_from_tag, referencia);
		robot_from_world.print("robot_from_world");

		///
		/// Compute new R
		///
		QMat tag_to_world_TR = innerModel->getTransformationMatrix(referencia, "world");
		QMat tag_to_world_rotation = QMat(3,3);
		for (int row=0; row<3; row++)
		{
		    for (int col=0; col<3; col++)
		    {
				tag_to_world_rotation(row, col) = tag_to_world_TR(row, col);
		    }
		}
		float newAngle = tag_to_world_rotation.extractAnglesR_min()(1) + robot_angle_from_tag;
		while (newAngle>+M_PIl) newAngle -= 2.*M_PIl;
		while (newAngle<-M_PIl) newAngle += 2.*M_PIl;
		printf("robot_ANGLE_from_world: %f\n", newAngle);

		///
		/// Send corrected odometry to the robot platform
		///
		try
		{
			differentialrobot_proxy->setOdometerPose(robot_from_world(0), robot_from_world(2), newAngle);
		}
		catch( Ice::Exception e)
		{
			fprintf(stderr, "Can't connect to DifferentialRobot\n");
		}
	//}
}
