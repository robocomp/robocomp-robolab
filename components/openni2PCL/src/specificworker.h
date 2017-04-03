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

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

#include <pcl/common/angles.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <boost/chrono.hpp>

#include <pcl/io/openni2/openni.h>



template <typename PointType> class OpenNI2Viewer
{
public:
	bool image_init, cloud_init;
	boost::signals2::connection cloud_connection;
	boost::signals2::connection image_connection;
	typedef pcl::PointCloud<PointType> Cloud;
	typedef typename Cloud::ConstPtr CloudConstPtr;

	OpenNI2Viewer (pcl::io::OpenNI2Grabber& grabber) : grabber_ (grabber), rgb_data_ (0), rgb_data_size_ (0)
	{
		readIM = new ImageMap();
		writeIM = new ImageMap();
		readPC = new PointCloudMap();
		writePC = new PointCloudMap();

		image_init = false;
		cloud_init = false;
		boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&OpenNI2Viewer::cloud_callback, this, _1);
		cloud_connection = grabber_.registerCallback (cloud_cb);
		if (grabber_.providesCallback<void (const boost::shared_ptr<pcl::io::openni2::Image>&)>())
		{
			boost::function<void (const boost::shared_ptr<pcl::io::openni2::Image>&) > image_cb = boost::bind (&OpenNI2Viewer::image_callback, this, _1);
			image_connection = grabber_.registerCallback (image_cb);
		}
		else
		{
			printf("Not providing image callbacks\n");
			exit(-1);
		}
		grabber_.start ();
	}

	void cloud_callback (const CloudConstPtr& cloud)
	{
		static bool first = true;
		boost::mutex::scoped_lock lock (cloud_mutex_);
		static PointCloud pc;
printf("%d\n", __LINE__);
		printf("%d\n", __LINE__);
		if (cloud != NULL)
		{
			printf("%d\n", __LINE__);
			if (pc.size() != cloud->points.size())
			{
				printf("%d\n", __LINE__);
				pc.resize(cloud->points.size());
			}
			printf("%d\n", __LINE__);
			for (int i=0; cloud->points.size(); i++)
			{
				pc[i].x = cloud->points[i].x;
				pc[i].y = cloud->points[i].y;
				pc[i].z = cloud->points[i].z;
				pc[i].r = cloud->points[i].r;
				pc[i].g = cloud->points[i].g;
				pc[i].b = cloud->points[i].b;
			}
			printf("%d\n", __LINE__);
		}
		printf("%d\n", __LINE__);
		// cloud_ = cloud;
		printf("%d\n", __LINE__);

		printf("%d\n", __LINE__);
		writePC->operator[]("rgbd") = pc;
		printf("%d\n", __LINE__);

		if (first)
		{
			printf("%d\n", __LINE__);
			*readPC = *writePC;
			first = false;
			printf("%d\n", __LINE__);
		}
		printf("%d\n", __LINE__);
		std::swap(readPC, writePC);
		printf("%d\n", __LINE__);
	}

	void image_callback (const boost::shared_ptr<pcl::io::openni2::Image>& image)
	{
		static bool first=true;
		static Image img;
		if (image->getEncoding () != pcl::io::openni2::Image::RGB)
		{
			printf("%d\n", __LINE__);
			if (rgb_data_size_ < image->getWidth () * image->getHeight ())
			{
				if (rgb_data_)
				{
					delete [] rgb_data_;
				}
				rgb_data_size_ = image->getWidth () * image->getHeight ();
				rgb_data_ = new unsigned char [rgb_data_size_ * 3];
			}
			image->fillRGB (image->getWidth(), image->getHeight(), rgb_data_);
			if (img.colorImage.size() != image->getWidth()*image->getHeight())
			{
				img.colorImage.resize(image->getWidth()*image->getHeight());
			}
			memcpy(&img.colorImage[0], rgb_data_, image->getWidth()*image->getHeight());
		}
		else
		{
			if (img.colorImage.size() != image->getWidth()*image->getHeight())
			{
				img.colorImage.resize(image->getWidth()*image->getHeight());
			}
			memcpy(&img.colorImage[0], (const unsigned char*)image->getData(), image->getWidth()*image->getHeight());
		}
		img.width = image->getWidth();
		img.height = image->getHeight();

		image_mutex_.lock();
		printf("cb %s %d\n", "rgbd", (int)img.colorImage.size());
		sleep(1);
		writeIM->operator[]("rbgd") = img;
		if (first)
		{
			*readIM = *writeIM;
			first = false;
		}
		std::swap(readIM, writeIM);
		image_mutex_.unlock();
	}

	void run ()
	{
	}

	pcl::io::OpenNI2Grabber& grabber_;
	boost::mutex cloud_mutex_;
	boost::mutex image_mutex_;

	// CloudConstPtr cloud;
	unsigned char *rgb_data_;
	unsigned rgb_data_size_;

	ImageMap *readIM, *writeIM;
	PointCloudMap *readPC, *writePC;

public:
	ImageMap *getImageMap()
	{
		return readIM;
	}
	PointCloudMap *getPointCloudMap()
	{
		return readPC;
	}
};



class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	CameraParamsMap getAllCameraParams();
	void getPointClouds(const CameraList &cameras, PointCloudMap &clouds);
	void getImages(const CameraList &cameras, ImageMap &images);
	void getProtoClouds(const CameraList &cameras, PointCloudMap &protoClouds);
	void getDecimatedImages(const CameraList &cameras, const int decimation, ImageMap &images);

public slots:
	void compute();

private:
	InnerModel *innerModel;


private:
	CameraParams params;
	pcl::io::OpenNI2Grabber *grabber;
	OpenNI2Viewer<pcl::PointXYZRGBA> *openni_viewer;

};

#endif
