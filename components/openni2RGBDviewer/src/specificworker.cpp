/*
 *    Copyright (C) 2017 by YOUR NAME HERE
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
	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor(0.2, 0.2, 0.2);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->points.resize(640*480);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	timer.start(Period);


	return true;
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	RoboCompRGBD::ColorSeq rgbMatrix;
	RoboCompRGBD::depthType distanceMatrix;
	RoboCompRGBD::PointSeq points;
	RoboCompJointMotor::MotorStateMap h;
	RoboCompGenericBase::TBaseState b;

	rgbd_proxy->getImage(rgbMatrix, distanceMatrix, points,  h, b);
	printf("%lu %lu\n", rgbMatrix.size(), points.size());
	cloud->points.resize(points.size());
	
	float accD=0;
	int accI=0;
	for (unsigned int i=0; i<points.size(); i++)
	{
		const float div = 1.;
		cloud->points[i].x = points[i].x/div;
		cloud->points[i].y = points[i].y/div;
		cloud->points[i].z = points[i].z/div;
		if (not isnan(cloud->points[i].z))
		{
			accD += cloud->points[i].z;
			accI += 1;
		}
		cloud->points[i].r=rgbMatrix[i].red;
		cloud->points[i].g=rgbMatrix[i].green;
		cloud->points[i].b=rgbMatrix[i].blue;
	}
	cloud->width = 1;
	cloud->height = points.size();
	printf("%f\n", accD/accI);

	cloud->is_dense = false;
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
	viewer->spinOnce(100);
}
