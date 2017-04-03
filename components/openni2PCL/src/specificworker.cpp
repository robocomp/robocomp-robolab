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
	double a,b;
	this->params.colorWidth = 640;
	this->params.colorHeight = 480;
	this->params.colorFPS = grabber->getFramesPerSecond();
	this->params.depthWidth = 640;
	this->params.depthHeight = 480;
	this->params.depthFPS = grabber->getFramesPerSecond();
	grabber->getRGBFocalLength(a, b);
	this->params.colorFocal = (a+b)/2.;
	grabber->getDepthFocalLength(a, b);
	this->params.depthFocal = (a+b)/2.;
	this->params.name = "rgbd";

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

CameraParamsMap SpecificWorker::getAllCameraParams()
{
	CameraParamsMap ret;
	ret[params.name] = params;
	return ret;
}

void SpecificWorker::getPointClouds(const CameraList &cameras, PointCloudMap &clouds)
{
	PointCloud pc;

	openni_viewer->cloud_mutex_.lock();
	if (openni_viewer->cloud != NULL)
  {
		pc.resize(openni_viewer->cloud->points.size());
		for (int i=0; openni_viewer->cloud->points.size(); i++)
		{
			pc[i].x = openni_viewer->cloud->points[i].x;
			pc[i].y = openni_viewer->cloud->points[i].y;
			pc[i].z = openni_viewer->cloud->points[i].z;
			pc[i].r = openni_viewer->cloud->points[i].r;
			pc[i].g = openni_viewer->cloud->points[i].g;
			pc[i].b = openni_viewer->cloud->points[i].b;
		}
	}
	openni_viewer->cloud_mutex_.unlock();

	clouds["rgb"] = pc;
}

void SpecificWorker::getImages(const CameraList &cameras, ImageMap &images)
{
	openni_viewer->image_mutex_.lock();
	images = *openni_viewer->getImageMap();
	openni_viewer->image_mutex_.unlock();
	for (auto &img : images)
	{
		img.second.camera = params;
		printf("getImages: %s: %d\n", img.first.c_str(), (int)img.second.colorImage.size());
	}
}

void SpecificWorker::getProtoClouds(const CameraList &cameras, PointCloudMap &protoClouds)
{
//implementCODE

}

void SpecificWorker::getDecimatedImages(const CameraList &cameras, const int decimation, ImageMap &images)
{
//implementCODE

}
