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
    
	    
    //leer el fichero de líneas
    // InnerModelPlane *plano = InnerModelPlane("id", );
//     innerModel->getNode("floor")->addChild(InnerModelPlane("id", ));
    
  
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
      loadLines();

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

		//updatePointCloud2(filteredPointCloud,polygons);
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
// 	printf("%s %d\n",__FUNCTION__, __LINE__);
	InnerModelNode *parent = innerViewer->innerModel->getNode(base);
	if (parent == NULL)
	{
		printf("%s: parent not exists\n", __FUNCTION__);
		return false;
	}
// 	qDebug() << __PRETTY_FUNCTION__<< __LINE__;

	
	
	try
	{
	  InnerModelPlane *plane = innerViewer->innerModel->newPlane(item, parent, texture, size(0), size(1), size(2), 1, n(0), n(1), n(2), p(0), p(1), p(2));
	  parent->addChild(plane);
// 	qDebug() << __PRETTY_FUNCTION__<< __LINE__;
	  
	  innerViewer->recursiveConstructor(plane, innerViewer->mts[parent->id], innerViewer->mts, innerViewer->meshHash);
// 	  qDebug() << __PRETTY_FUNCTION__<< __LINE__;
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
// addPlane_notExisting(InnerModelViewer *innerViewer, const QString &item, const QString &base, const QVec &p, const QVec &n, const QString &texture, const QVec &size)
	imvPointCloud->points->clear();
	imvPointCloud->colors->clear();

/*	for(unsigned int i; i<polygons.size() && i%2==0;i++) {
	      if (innerModelViewer->innerModel->getNode("poly"+i))
	      {
 		      //removeNode(innerModelViewer, "poly"+i);
		      qDebug() << "remove node";
	      }
	      else {
 		addPlane_notExisting(innerModelViewer,"poly"+i,"floor",QVec::vec3(polygons[i].p0.x,polygons[i].p0.y,polygons[i].p0.z),QVec::vec3(polygons[i].normal.x,polygons[i].normal.y,polygons[i].normal.z), 			"#00A0A0",QVec::vec3(polygons[i].width,polygons[i].height,200));
   	      }
	}
*/

	QMat m = innerModel->getTransformationMatrix("world","rgbd");
	
	for (size_t i = 0; i < points.size (); i++)
	{
		osg::Vec3f p = QVecToOSGVec( m * QVec::vec4(points[i].x, points[i].y, points[i].z,1.f));
		imvPointCloud->points->push_back(p);
		imvPointCloud->colors->push_back( osg::Vec4( 1.,  0.,  0.,  1 ) );
	}
	imvPointCloud->update();	
}
RoboCompRGBD::PointSeq SpecificWorker::getFilteresPoints()
{
	//QMutexLocker l(pointsMutex);
	return points;
}
void SpecificWorker::loadLines()
{
	char buffer[100];
	char *ptr;
	ifstream fichero;
	int i =0;
	float width;
	QVec p1,p2;
	fichero.open("points");
	fichero.getline(buffer,100,'\n');
	ptr = strtok(buffer, " ");
	while(ptr!=NULL)
	{
		for(int j=0;j<3;j++)
		{
			p1(j) = atof(ptr);
			ptr = strtok(NULL, " ");
		}
	}
	fichero.getline(buffer,100,'\n');
	while(!fichero.eof())
	{
		
		ptr = strtok(buffer, " ");
		while(ptr!=NULL)
		{
			for(int j=0;j<3;j++)
			{
				p2(j) = atof(ptr);
				ptr = strtok(NULL, " ");
			}
		}
		qDebug()<<p2(0)<<p2(1)<<p2(2);
		QVec n = QVec::vec3(p2(0)-p1(0),p2(1)-p1(1),p2(2)-p1(2));
		width = (QVec::vec2(p2(0)-p1(0),p2(2)-p1(2))).norm2();
		addPlane_notExisting(innerModelViewer,"LINEA_"+i,"floor",QVec::vec3((p1(0)+p2(0))/2,0,(p1(2)+p2(2))/2),QVec::vec3(-n(2),0,n(0)),"#00A0A0",
		QVec::vec3(width, 100, 100));
		i++;
		p1(0)=p2(0);
		p1(2)=p2(2);
		fichero.getline(buffer,100,'\n');
	}
	fichero.close();

}

// void SpecificWorker::updatePointCloud2( const vector< vector3f > &points,vector< PlanePolygon > polygons)
// {
// 	imvPointCloud->points->clear();
// 	imvPointCloud->colors->clear();
// 	for(unsigned int i; i<polygons.size();i++) {
// 	  InnerModelNode *parent = innerModelViewer->innerModel->getNode("floor");
// 			static bool addPlane_ignoreExisting(InnerModelViewer *innerViewer, const QString &a, const QString &b, const QVec &p, const QVec &n, const QString &texture, const QVec &size);
// 	  InnerModelPlane *plane = innerModelViewer->innerModel->newPlane("polys"+i, parent, "#00A0A0", polygons[i].width, polygons[i].height, 100, 1, polygons[i].normal.x, polygons[i].normal.y, polygons[i].normal.z, polygons[i].p0.x, polygons[i].p0.y, polygons[i].p0.z);
// 	  parent->addChild(plane);
// 	  innerModelViewer->recursiveConstructor(plane,innerModelViewer->mts[parent->id],innerModelViewer->mts,innerModelViewer->meshHash);
// 	}
// 	
// 	QMat m = innerModel->getTransformationMatrix("world","rgbd");
// 	
// 	for (size_t i = 0; i < points.size (); i++)
// 	{
// 		osg::Vec3f p = QVecToOSGVec( m * QVec::vec4(points[i].x, points[i].y, points[i].z,1.f));
// 		imvPointCloud->points->push_back(p);
// 		imvPointCloud->colors->push_back( osg::Vec4( 1.,  0.,  0.,  1 ) );
// 	}
// 	imvPointCloud->update();
// }
