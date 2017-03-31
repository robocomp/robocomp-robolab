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

#include <pcl/io/pcd_io.h>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	std::string device_id ("");
	pcl::io::OpenNI2Grabber::Mode depth_mode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
	pcl::io::OpenNI2Grabber::Mode image_mode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;

	boost::shared_ptr<pcl::io::openni2::OpenNI2DeviceManager> deviceManager = pcl::io::openni2::OpenNI2DeviceManager::getInstance ();
	if (deviceManager->getNumOfConnectedDevices () > 0)
	{
	  boost::shared_ptr<pcl::io::openni2::OpenNI2Device> device = deviceManager->getAnyDevice ();
	  cout << "Device ID not set, using default device: " << device->getStringID () << endl;
	}

	grabber = new pcl::io::OpenNI2Grabber(device_id, depth_mode, image_mode);

	// if (!grabber->providesCallback<pcl::io::OpenNI2Grabber::sig_cb_openni_point_cloud_rgb> ())
	// {
	//   printf("a\n");
	//   OpenNI2Viewer<pcl::PointXYZ> openni_viewer(*grabber);
	//   openni_viewer.run();
	// }
	// else
	{
	  printf("b\n");
	  openni_viewer = new OpenNI2Viewer<pcl::PointXYZRGBA>(*grabber);
	  openni_viewer->run();
	}
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

	openni_viewer->run();

	// openni_viewer->cloud_mutex_.lock();
	// if (openni_viewer->cloud != NULL)
  // {
	// 	static int i=0;
	// 	pcl::io::savePCDFileASCII ("pcd"+std::to_string(i++)+".pcd", *openni_viewer->cloud);
	// }
	// else
	// {
	// 	printf(".");
	// 	fflush(stdout);
	// }
	// openni_viewer->cloud_mutex_.unlock();

}


Registration SpecificWorker::getRegistration()
{
	return ColorInDepth;
}

void SpecificWorker::getData(imgType &rgbMatrix, depthType &distanceMatrix, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState)
{
	openni_viewer->image_mutex_.lock();
	rgbMatrix.resize(openni_viewer->rgb_data_size_);
	memcpy(&rgbMatrix[0], openni_viewer->rgb_data_, openni_viewer->rgb_data_size_);
	openni_viewer->image_mutex_.unlock();
}

void SpecificWorker::getXYZ(PointSeq &points, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState)
{
	openni_viewer->cloud_mutex_.lock();
	if (openni_viewer->cloud != NULL)
  {
		points.resize(openni_viewer->cloud->points.size());
		for (int i=0; openni_viewer->cloud->points.size(); i++)
		{
			points[i].x = openni_viewer->cloud->points[i].x;
			points[i].y = openni_viewer->cloud->points[i].y;
			points[i].z = openni_viewer->cloud->points[i].z;
		}
	}
	openni_viewer->cloud_mutex_.unlock();
}

void SpecificWorker::getRGB(ColorSeq &color, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState)
{
	openni_viewer->image_mutex_.lock();
	color.resize(openni_viewer->rgb_data_size_/3);
	memcpy(&color[0], openni_viewer->rgb_data_, openni_viewer->rgb_data_size_);
	openni_viewer->image_mutex_.unlock();
//implementCODE

}

TRGBDParams SpecificWorker::getRGBDParams()
{
//implementCODE

}

void SpecificWorker::getDepth(DepthSeq &depth, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState)
{
//implementCODE

}

void SpecificWorker::setRegistration(const Registration &value)
{
//implementCODE

}

void SpecificWorker::getImage(ColorSeq &color, DepthSeq &depth, PointSeq &points, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState)
{
//implementCODE

}

void SpecificWorker::getDepthInIR(depthType &distanceMatrix, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState)
{
//implementCODE

}
