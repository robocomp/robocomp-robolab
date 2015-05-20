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
	innerModel = new InnerModel("world.xml");
	
	osgView = new OsgView (frame);
	// 	osgView->setCameraManipulator(new osgGA::TrackballManipulator); 	
	//osgView->getCameraManipulator()->setHomePosition(osg::Vec3(0.,0.,-2.),osg::Vec3(0.,0.,4.),osg::Vec3(0.,1,0.));
	innerModelViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup());
	imvPointCloud = innerModelViewer->pointCloudsHash["cloud"];
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//
// 	try
// 	{
// 		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
// 		innermodel_path=par.value;
// 		innermodel = new InnerModel(innermodel_path);
// 	}
// 	catch(std::exception e) { qFatal("Error reading config params"); }
	
	
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
 	try
 	{
		static RoboCompDifferentialRobot::TBaseState bState;
		static RoboCompJointMotor::MotorStateMap hState;
		RoboCompRGBD::PointSeq points;		
		
 		rgbd_proxy->getXYZ(points, hState, bState);
		qDebug() << points.size();
		updatePointCloud(points);
		
 	}
 	catch(const Ice::Exception &e)
 	{
 		std::cout << "Error reading from Camera" << e << std::endl;
 	}
 	 	
 	innerModelViewer->update();
	osgView->autoResize();
	osgView->frame();
}


void SpecificWorker::updatePointCloud(const RoboCompRGBD::PointSeq &points)
{
	imvPointCloud->points->resize(points.size());
	imvPointCloud->colors->resize(points.size());
	for (size_t i = 0; i < points.size (); ++i)
	{
		imvPointCloud->points->operator[](i) = QVecToOSGVec(QVec::vec3(
		  points[i].x,
		  points[i].y,
		  points[i].z)
		);
 		imvPointCloud->colors->operator[](i) = osg::Vec4(
 		  1.,
 		  0.,
 		  0.,
 		  1
 		);
	}
	imvPointCloud->update();

}


