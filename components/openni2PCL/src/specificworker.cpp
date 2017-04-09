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
	bStateMutex = new QMutex();
	mStateMutex = new QMutex();
	talkToJoint = false;
	talkToBase = false;


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


	if (!grabber->providesCallback<pcl::io::OpenNI2Grabber::sig_cb_openni_point_cloud_rgb> ())
	{
	  OpenNI2Viewer<pcl::PointXYZ> openni_viewer(*grabber);
	  openni_viewer.run();
	}
	else
	{
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
	talkToJoint = QString::fromStdString(params["TalkToJoint"].value).contains("true");
	talkToBase = QString::fromStdString(params["TalkToBase"].value).contains("true");

	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	openni_viewer->run();

	if (talkToBase)
	{
		bStateMutex->lock();
		try
		{
			genericbase_proxy->getBaseState(bState);
		}
		catch(...)
		{
			qDebug()<<"Exception: error reading genericbase state";
		}
		bStateMutex->unlock();
	}
	if (talkToJoint)
	{
		mStateMutex->lock();
		try
		{
			jointmotor_proxy->getAllMotorState(mState);
		}
		catch(...)
		{
			qDebug()<<"Exception: error reading motorStates";
		}
		mStateMutex->unlock();

	}
}


Registration SpecificWorker::getRegistration()
{
//implementCODE

}

void SpecificWorker::getData(imgType &rgbMatrix, depthType &distanceMatrix, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState)
{
//implementCODE

}

void SpecificWorker::getXYZ(PointSeq &points, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState)
{
	openni_viewer->cloud_mutex_.lock();
	points = *openni_viewer->getCloud();
	openni_viewer->cloud_mutex_.unlock();

	mStateMutex->lock();
	hState = mState;
	mStateMutex->unlock();

	bStateMutex->lock();
	bState = this->bState;
	bStateMutex->unlock();
}

void SpecificWorker::getRGB(ColorSeq &color, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState)
{
	openni_viewer->image_mutex_.lock();
	color = *openni_viewer->getImage();
	openni_viewer->image_mutex_.unlock();

	
	mStateMutex->lock();
	hState = mState;
	mStateMutex->unlock();

	bStateMutex->lock();
	bState = this->bState;
	bStateMutex->unlock();
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
	openni_viewer->cloud_mutex_.lock();
	points = *openni_viewer->getCloud();
	openni_viewer->cloud_mutex_.unlock();

	openni_viewer->image_mutex_.lock();
	color = *openni_viewer->getImage();
	openni_viewer->image_mutex_.unlock();

	
	mStateMutex->lock();
	hState = mState;
	mStateMutex->unlock();

	bStateMutex->lock();
	bState = this->bState;
	bStateMutex->unlock();
}

void SpecificWorker::getDepthInIR(depthType &distanceMatrix, RoboCompJointMotor::MotorStateMap &hState, RoboCompGenericBase::TBaseState &bState)
{
//implementCODE

}
