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
    innerModel = new InnerModel("etc/world.xml");
    
	    

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
    
//     PlaneFilter::PlaneFilterParams filterParams;
//     
//     filterParams.maxPoints = 2000;
//     filterParams.numSamples = 20000;    
//     filterParams.numLocalSamples = 80;
//     filterParams.planeSize = 500;
//     filterParams.WorldPlaneSize = 20;
//     filterParams.minInlierFraction = 0.8;
//     
//     filterParams.maxError = 10;
//     filterParams.numRetries = 2;
//   
//     filterParams.maxDepthDiff = 1800;
//     
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
		
		vector< vector3f > filteredPointCloud, pointCloudNormals, outlierCloud;
		vector< vector2i > pixelLocs;
		vector< PlanePolygon > polygons;
		
		planeFilter->GenerateFilteredPointCloud(points, filteredPointCloud, pixelLocs, pointCloudNormals, outlierCloud, polygons);

		updatePointCloud2(filteredPointCloud,polygons);
		qDebug() << points.size() << filteredPointCloud.size() << outlierCloud.size();

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
		osg::Vec3f p = QVecToOSGVec( m * QVec::vec4(points[i].x, 10/*points[i].y*/, points[i].z,1.f));
		imvPointCloud->points->push_back(p);
		imvPointCloud->colors->push_back( osg::Vec4( 1.,  0.,  0.,  1 ) );

	}
	imvPointCloud->update();
}

bool SpecificWorker::addPlane_notExisting(InnerModelViewer *innerViewer, const QString &item, const QString &base, const QVec &p, const QVec &n, const QString &texture, const QVec &size)
{
	InnerModelNode *parent = innerViewer->innerModel->getNode(base);
	if (parent == NULL)
	{
		printf("%s: parent not exists\n", __FUNCTION__);
		return false;
	}
	
	try
	{
	  InnerModelPlane *plane = innerViewer->innerModel->newPlane(item, parent, texture, size(0), size(1), size(2), 1, n(0), n(1), n(2), p(0), p(1), p(2));
	  parent->addChild(plane);
	  
	  innerViewer->recursiveConstructor(plane, innerViewer->mts[parent->id], innerViewer->mts, innerViewer->meshHash);

	}
	catch (QString err)
	{
		printf("%s:%s:%d: Exception: %s\n", __FILE__, __FUNCTION__, __LINE__, err.toStdString().c_str());
		throw;
	}		
	return true;
}


bool SpecificWorker::removeNode(InnerModelViewer *innerViewer, const QString &item)
{
	if (item=="floor")
	{
		qDebug() << "Can't remove root elements" << item;
		return false;
	}

	InnerModelNode *node = innerViewer->innerModel->getNode(item);
	if (node == NULL)
	{
		qDebug() << "Can't remove not existing elements" << item;
		return false;
	}

	QStringList l;
	innerViewer->innerModel->getSubTree(node, &l);
	innerViewer->innerModel->removeSubTree(node, &l);

	/// Replicate InnerModel node removals in the InnerModelViewer tree. And in handlers Lists
	foreach(QString n, l)
	{
		/// Replicate mesh removals
		if (innerViewer->meshHash.contains(n))
		{
			while (innerViewer->meshHash[n].osgmeshPaths->getNumParents() > 0)
				innerViewer->meshHash[n].osgmeshPaths->getParent(0)->removeChild(innerViewer->meshHash[n].osgmeshPaths);			
			while(innerViewer->meshHash[n].osgmeshes->getNumParents() > 0)
				innerViewer->meshHash[n].osgmeshes->getParent(0)->removeChild(innerViewer->meshHash[n].osgmeshes);
			while(innerViewer->meshHash[n].meshMts->getNumParents() > 0)	
				innerViewer->meshHash[n].meshMts->getParent(0)->removeChild(innerViewer->meshHash[n].meshMts);			
			innerViewer->meshHash.remove(n);
		}
		/// Replicate transform removals
		if (innerViewer->mts.contains(n))
		{
 			while (innerViewer->mts[n]->getNumParents() > 0)
				innerViewer->mts[n]->getParent(0)->removeChild(innerViewer->mts[n]);
 			innerViewer->mts.remove(n);
		}
		/// Replicate plane removals
		if (innerViewer->planeMts.contains(n))
		{
			while(innerViewer->planeMts[n]->getNumParents() > 0)
				((osg::Group *)(innerViewer->planeMts[n]->getParent(0)))->removeChild(innerViewer->planeMts[n]);
			innerViewer->planeMts.remove(n);
			innerViewer->planesHash.remove(n);
		}
		
	}

	return true;
}

void SpecificWorker::updatePointCloud2( const vector< vector3f > &points,vector< PlanePolygon > polygons)
{
     // TODO draw polygons.
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

