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
	viewer->setBackgroundColor (0, 0, 0);
     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
     viewer->addCoordinateSystem (1.0);
     viewer->initCameraParameters ();
     return (viewer);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	RoboCompRGBD::ColorSeq rgbMatrix;
	RoboCompRGBD::depthType distanceMatrix;
	RoboCompRGBD::PointSeq points_kinect;
	RoboCompJointMotor::MotorStateMap h;
	RoboCompGenericBase::TBaseState b;

	rgbd_proxy->getImage(rgbMatrix, distanceMatrix, points_kinect,  h, b);
	cout<<"SpecificWorker::grabThePointcloud rgbMatrix.size(): "<<rgbMatrix.size()<<endl;
	for(unsigned int i=0; i<rgbMatrix.size(); i++)
	{
		int row = (i/640), column = i-(row*640);
		rgb_image.at<cv::Vec3b>(row, column) = cv::Vec3b(rgbMatrix[i].blue, rgbMatrix[i].green, rgbMatrix[i].red);
	}
	viewpoint_transform = innermodel->getTransformationMatrix(id_robot,id_camera_transform);
	QMat PP = viewpoint_transform;
	cloud->points.resize(points_kinect.size());
	for (unsigned int i=0; i<points_kinect.size(); i++)
	{
		QVec p1 = (PP * QVec::vec4(points_kinect[i].x, points_kinect[i].y, points_kinect[i].z, 1)).fromHomogeneousCoordinates();

		memcpy(&cloud->points[i],p1.data(),3*sizeof(float));

		cloud->points[i].r=rgbMatrix[i].red;
		cloud->points[i].g=rgbMatrix[i].green;
		cloud->points[i].b=rgbMatrix[i].blue;
	}
	cloud->width = 1;
	cloud->height = points_kinect.size();
// 		Convert cloud from mm to m
	if(MEDIDA==1000.)
		cloud = PointCloudfrom_mm_to_Meters(cloud);

	std::vector< int > index;
	removeNaNFromPointCloud (*cloud, *cloud, index);
	cloud->is_dense = false;
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");




	timer.start(Period);


	return true;
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	//computeCODE
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
}
