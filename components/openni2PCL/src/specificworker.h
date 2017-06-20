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
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/boost.h>
//#include <pcl/visualization/image_viewer.h>
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
		readIM = new ColorSeq();
		readIM->resize(640*480);
		writeIM = new ColorSeq();
		writeIM->resize(640*480);
		readCM = new PointSeq();
		readCM->resize(640*480);
		writeCM = new PointSeq();
		writeCM->resize(640*480);

		image_init = false;
		cloud_init = false;
		boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&OpenNI2Viewer::cloud_callback, this, _1);
		cloud_connection = grabber_.registerCallback (cloud_cb);
		if (grabber_.providesCallback<void (const boost::shared_ptr<pcl::io::openni2::Image>&)>())
		{
			boost::function<void (const boost::shared_ptr<pcl::io::openni2::Image>&) > image_cb = boost::bind (&OpenNI2Viewer::image_callback, this, _1);
			image_connection = grabber_.registerCallback (image_cb);
		}
		grabber_.start ();
	}

	void cloud_callback (const CloudConstPtr& cloud)
	{
		boost::mutex::scoped_lock lock (cloud_mutex_);
		cloud_ = cloud;

		static bool first=true;

		if (writeCM->size() != cloud->points.size())
			writeCM->resize(cloud->points.size());

		for (uint32_t i=0; i<cloud->points.size(); i++)
		{
			writeCM->operator[](i).x =  1000. * cloud->points[i].x;
			writeCM->operator[](i).y = -1000. * cloud->points[i].y;
			writeCM->operator[](i).z =  1000. * cloud->points[i].z;
			writeCM->operator[](i).w = 1;
		}
		if (first)
		{
			*readCM = *writeCM;
			first = false;
		}

		PointSeq *t = readCM;
		readCM = writeCM;
		writeCM = t;
	}

	void image_callback (const boost::shared_ptr<pcl::io::openni2::Image>& image)
	{
		static bool first=true;
		boost::mutex::scoped_lock lock (image_mutex_);
		image_ = image;

		if (image->getEncoding () != pcl::io::openni2::Image::RGB)
		{
			printf("%d\n", __LINE__);
			if (rgb_data_size_ < image->getWidth () * image->getHeight ())
			{
			printf("%d\n", __LINE__);
				if (rgb_data_)
					delete [] rgb_data_;
				rgb_data_size_ = image->getWidth () * image->getHeight ();
				rgb_data_ = new unsigned char [rgb_data_size_ * 3];
			}
			printf("%d\n", __LINE__);
			image_->fillRGB (image_->getWidth (), image_->getHeight (), rgb_data_);
		}


//     boost::mutex::scoped_lock lock (image_mutex_);
// 		image_ = image;
//
// 		if (image->getEncoding () != pcl::io::openni2::Image::RGB)
// 		{
// 			if (rgb_data_size_ < image->getWidth () * image->getHeight ())
// 			{
// 				if (rgb_data_)
// 					delete [] rgb_data_;
// 				rgb_data_size_ = image->getWidth () * image->getHeight ();
// 				rgb_data_ = new unsigned char [rgb_data_size_ * 3];
// 			}
// 			image_->fillRGB (image_->getWidth (), image_->getHeight (), rgb_data_);
// 			printf("d2\n");
// 		}
// //
// 		uint8_t *rgb_xxx = new unsigned char[640*480*3];
// 		printf("%p %dx%d\n", image_->getData(), image->getWidth(), image->getHeight());
// 		memcpy(rgb_xxx, (const unsigned char*)image_->getData(), image->getWidth()*image->getHeight());

		printf("%p %dx%d\n", image_->getData(), image->getWidth(), image->getHeight());
		memcpy(&writeIM->operator[](0), (const unsigned char*)image_->getData(), image->getWidth()*image->getHeight()*3);

		if (first)
		{
			*readIM = *writeIM;
			first = false;
		}
		ColorSeq *t = readIM;
		readIM = writeIM;
		writeIM = t;
	}


	void run ()
	{
		boost::shared_ptr<pcl::io::openni2::Image> image;
		// See if we can get a cloud
		if (cloud_mutex_.try_lock ())
		{
			cloud_.swap (cloud);
			cloud_mutex_.unlock ();
		}

		if (cloud)
		{
			if (!cloud_init)
			{
				cloud_init = !cloud_init;
			}
			if (image_mutex_.try_lock ())
			{
				image_.swap(image);
				image_mutex_.unlock ();
			}
			if (image)
			{
				if (!image_init && cloud && cloud->width != 0)
				{
					image_init = !image_init;
				}
			}
		}
	}

public:
	pcl::io::OpenNI2Grabber &grabber_;
	boost::mutex cloud_mutex_;
	boost::mutex image_mutex_;

	CloudConstPtr cloud_, cloud;
	boost::shared_ptr<pcl::io::openni2::Image> image_;
	unsigned char* rgb_data_;
	unsigned rgb_data_size_;

	ColorSeq *readIM, *writeIM;
	PointSeq *readCM, *writeCM;

public:
	ColorSeq *getImage()
	{
		return readIM;
	}
	PointSeq *getCloud()
	{
		return readCM;
	}
};



class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	Registration getRegistration();
	void getData(imgType &rgbMatrix, depthType &distanceMatrix, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState);
	void getXYZ(PointSeq &points, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState);
	void getRGB(ColorSeq &color, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState);
	TRGBDParams getRGBDParams();
	void getDepth(DepthSeq &depth, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState);
	void setRegistration(const Registration &value);
	void getImage(ColorSeq &color, DepthSeq &depth, PointSeq &points, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState);
	void getDepthInIR(depthType &distanceMatrix, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState);

public slots:
	void compute();

private:
	InnerModel *innerModel;


private:
	pcl::io::OpenNI2Grabber *grabber;
	OpenNI2Viewer<pcl::PointXYZRGBA> *openni_viewer;

	RoboCompGenericBase::TBaseState bState;
	RoboCompJointMotor::MotorStateMap mState;
	QMutex *bStateMutex, *mStateMutex;
	bool talkToJoint,talkToBase;


};

#endif
