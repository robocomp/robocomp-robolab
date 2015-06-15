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

		vector< vector3f > filteredPointCloud,pointsNormals , outlierCloud;
		vector< vector2i > pixelLocs;
		vector< PlanePolygon > polygons;

		planeFilter->GenerateFilteredPointCloud(points, filteredPointCloud, pixelLocs, pointsNormals, outlierCloud, polygons);
		
		RoboCompFSPF::OrientedPoints ops;
		for( int i =0; i<filteredPointCloud.size(); i++)
		{
			RoboCompFSPF::OrientedPoint op;
			op.x = filteredPointCloud[i].x;
			op.y = filteredPointCloud[i].y;
			op.z = filteredPointCloud[i].z;
			op.nx = pointsNormals[i].x;
			op.ny = pointsNormals[i].y;
			op.nz = pointsNormals[i].z;
			ops.push_back(op);		
		}

//		qDebug() << "ops" << ops.size();
		fspf_proxy->newFilteredPoints(ops);

		qDebug() << points.size() << filteredPointCloud.size() << outlierCloud.size();
 	}
 	catch(const Ice::Exception &e)
 	{
 		std::cout << "Error reading from Camera" << e << std::endl;
 	}
	co++;
	if( reloj.elapsed() > 1000)
	{
	  qDebug() << co << " fps";
	  co = 0;
	  reloj.restart();
	}
}


//////////////////////////77
/// SERVANT
///////////////////////////


