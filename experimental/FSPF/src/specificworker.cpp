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
  
//     PlaneFilter::PlaneFilterParams filterParams;
//     filterParams.maxPoints = 2000;
//     filterParams.numSamples = 10000;
//     filterParams.numLocalSamples = 50;
//     filterParams.maxDepthDiff = 1800;    
//     filterParams.planeSize = 100;
//     filterParams.WorldPlaneSize = 50;
//     
//     filterParams.maxError = 30;
//     filterParams.minInlierFraction = 0.80;
//     filterParams.numRetries = 2;
//   
//     // Parameters for polygonization
//     filterParams.runPolygonization = false;
//     filterParams.minConditionNumber = 0.1;
    
    PlaneFilter::PlaneFilterParams filterParams;
    
    filterParams.maxPoints = 2000;
    filterParams.numSamples = 20000;    
    filterParams.numLocalSamples = 80;
    filterParams.planeSize = 100;
    filterParams.WorldPlaneSize = 50;
    filterParams.minInlierFraction = 0.8;
    
    filterParams.maxError = 20;
    filterParams.numRetries = 2;
  
    
    filterParams.maxDepthDiff = 1800;
    
    
    
    
    
    // Parameters for polygonization
    filterParams.runPolygonization = false;
    filterParams.minConditionNumber = 0.1;
    
    
    
    
    
    
    
    
    //Thresholds for polygon merging
    //double maxCosineError;
    //float maxPolygonDist;
    //float maxOffsetDiff;
    //float minVisibilityFraction;
    filterParams.filterOutliers = true;
	
    planeFilter = new PlaneFilter( points, filterParams);
	
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
	
	
	timer.start(0);
	return true;
}

void SpecificWorker::compute()
{
	static QTime reloj=QTime::currentTime();
	static int co = 0;
  
 	try
 	{
		static RoboCompDifferentialRobot::TBaseState bState;
		static RoboCompJointMotor::MotorStateMap hState;
		
 		rgbd_proxy->getXYZ(points, hState, bState);
		//updatePointCloud(points);
		
		vector< vector3f > filteredPointCloud, pointCloudNormals, outlierCloud;
		vector< vector2i > pixelLocs;
		vector< PlanePolygon > polygons;
		
		planeFilter->GenerateFilteredPointCloud(points, filteredPointCloud, pixelLocs, pointCloudNormals, outlierCloud, polygons);
		qDebug() << points.size() << filteredPointCloud.size() << outlierCloud.size();
		updatePointCloud2(filteredPointCloud);

 	}
 	catch(const Ice::Exception &e)
 	{
 		std::cout << "Error reading from Camera" << e << std::endl;
 	}
 	 	
 	innerModelViewer->update();
	osgView->autoResize();
	osgView->frame();

	co++;
	if( reloj.elapsed() > 1000)
	{
	  qDebug() << co << " fps";
	  co = 0;
	  reloj.restart();
	}
	
	
	
	
}

void SpecificWorker::updatePointCloud(const RoboCompRGBD::PointSeq &points)
{
	imvPointCloud->points->clear();
	imvPointCloud->colors->clear();
	
	QMat m = innerModel->getTransformationMatrix("world","rgbd");
	
	for (size_t i = 0; i < points.size (); i+=4)
	{
		osg::Vec3f p = QVecToOSGVec( m * QVec::vec4(points[i].x, points[i].y, points[i].z,1.f));
		imvPointCloud->points->push_back(p);
		imvPointCloud->colors->push_back( osg::Vec4( 1.,  0.,  0.,  1 ) );

	}
	imvPointCloud->update();
}

void SpecificWorker::updatePointCloud2( const vector< vector3f > &points)
{
	imvPointCloud->points->clear();
	imvPointCloud->colors->clear();
	
	QMat m = innerModel->getTransformationMatrix("world","rgbd");
	
	for (size_t i = 0; i < points.size (); i++)
	{
		osg::Vec3f p = QVecToOSGVec( m * QVec::vec4(points[i].x, points[i].y, points[i].z,1.f));
		imvPointCloud->points->push_back(p);
		imvPointCloud->colors->push_back( osg::Vec4( 1.,  0.,  0.,  1 ) );
	}
	imvPointCloud->update();
}










