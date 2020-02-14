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
	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
  osg::Vec3d eye(osg::Vec3(000.,3000.,-6000.));
  osg::Vec3d center(osg::Vec3(0.,0.,-0.));
  osg::Vec3d up(osg::Vec3(0.,1.,0.));
  tb->setHomePosition(eye, center, up, true);
  tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
  osgView->setCameraManipulator(tb);
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
	static bool firstTime = true;
 	try
 	{
		static RoboCompDifferentialRobot::TBaseState bState;
		static RoboCompJointMotor::MotorStateMap hState;
		RoboCompRGBD::PointSeq points;		
		
 		rgbd_proxy->getXYZ(points, hState, bState);
		qDebug() << points.size();
		
		if ( firstTime )
		{
			computeBackground(points);
			firstTime = false; 
		}
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

void SpecificWorker::computeBackground(RoboCompRGBD::PointSeq &points)
{
		static RoboCompDifferentialRobot::TBaseState bState;
		static RoboCompJointMotor::MotorStateMap hState;
		rgbd_proxy->getXYZ(points, hState, bState);
		rgbd_proxy->getXYZ(points, hState, bState);
	
		data.resize(3,points.size()/4);
		imvPointCloud->points->resize(points.size()/4);
		imvPointCloud->colors->resize(points.size()/4);
		int j=0;
		for (size_t i = 0; i < points.size (); i+=4)
		{
			osg::Vec3f d = QVecToOSGVec(innerModel->transform("world",QVec::vec3(points[i].x, points[i].y, points[i].z),"rgbd"));
			data( 0, j ) = d.x();
			data( 1, j ) = d.y();
			data( 2, j++ ) = d.z();
		}	
		nns = NNSearchF::createKDTreeLinearHeap( data );	
}

void SpecificWorker::storeBackground()
{
		
// 		VectorXi indices(K);
// 		VectorXf dists2(K);
// 		
// 		int j=0;
// 		for (size_t i = 0; i < points.size (); i+=4)
// 		{
// 			nns->knn( , indices, dist2, imvPointCloud );
// 		}
// 		
}



void SpecificWorker::updatePointCloud(const RoboCompRGBD::PointSeq &points)
{
	imvPointCloud->points->clear();
	imvPointCloud->colors->clear();
	for (size_t i = 0; i < points.size (); i+=4)
	{
		 osg::Vec3f p = QVecToOSGVec(innerModel->transform("world",QVec::vec3(points[i].x, points[i].y, points[i].z),"rgbd"));
		 if( filterP( p , points[i] ) )
		 {
				imvPointCloud->points->push_back(p);
				imvPointCloud->colors->push_back( osg::Vec4( 1.,  0.,  0.,  1 ) );
		 }
	}
	imvPointCloud->update();
}


bool SpecificWorker::filterP( const osg::Vec3f &p, const RoboCompRGBD::PointXYZ &point )
{
	const int K = 1;
	VectorXi indices(K);
	VectorXf dists2(K);

	VectorXf eq(3);
	eq(0) = p.x(); eq(1) = p.y(); eq(2) = p.z();
	//nns->knn(eq, indices, dists2, K);
	
	
	//if(   dists2.coeff(0) > ( point.z/2.f  ))
	if(   point.z < 3500 and p.y() > 1000)
		return true;
	else 
		return false;
}
