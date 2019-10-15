/*
 *    Copyright (C)2019 by YOUR NAME HERE
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
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{
	initialPose.set(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	serial = params["serial"].value;
	return true;
}

//workaround => using serial value not working on actual api version
rs2::device SpecificWorker::get_device(const std::string& serial_number) {
    rs2::context ctx;
    while (true) {
        for (auto&& dev : ctx.query_devices())
            if (std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) == serial_number)
                return dev;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	fullpose.source = "realsense";
	// Add pose stream
	try{
		cfg.enable_device(serial);
		cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
		// Start pipeline with chosen configuration
		pipe.start(cfg);
	}catch(...)
	{
		qFatal("Unable to open device, please check config file");
	}
	this->Period = 20;
	timer.start(Period);
}

void SpecificWorker::compute()
{
	auto frames = pipe.wait_for_frames();
    // Get a frame from the pose stream
    auto f = frames.first_or_default(RS2_STREAM_POSE);
	// Cast the frame to pose_frame and get its data
	auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
	// Print the x, y, z values of the translation, relative to initial position
	std::cout << "\r" << "Device Position: " << std::setprecision(3) 
			  << std::fixed << pose_data.translation.x << " " 
			  << pose_data.translation.y << " " 
			  << pose_data.translation.z << " (meters)" << " " 
			  << pose_data.rotation.x << " "
			  << pose_data.rotation.y << " "
			  << pose_data.rotation.z << " "
			  << pose_data.rotation.w << " (quat)";

	// Move the robot
	const auto &tr = pose_data.translation;
	const auto &rot = pose_data.rotation;
	RMat::Quaternion q(rot.x,rot.y,rot.z,rot.w);
	QVec angles = q.toAngles();
		
	RTMat cam(angles.x(), -angles.y(), angles.z(),
			  tr.x*1000, tr.y*1000, -tr.z*1000);
	std::cout << "X "<<tr.x<<std::endl;
	RTMat pose = initialPose * cam;
	QVec angles2 = pose.extractAnglesR();
	std::lock_guard<std::mutex> lock(bufferMutex);
	fullpose.x = pose.getTr().x();
	fullpose.y = pose.getTr().y();
	fullpose.z = pose.getTr().z();
	fullpose.rx = angles.rx();
	fullpose.ry = -angles.ry();
	fullpose.rz = angles.rz();

	//publish
	try
	{
		fullposeestimationpub_pubproxy->newFullPose(fullpose);
	}catch(const Ice::Exception& ex)
	{
		std::cout << "Exception publishing pose: "<<ex << std::endl;
	}
	
}


FullPose SpecificWorker::FullPoseEstimation_getFullPose()
{
	std::lock_guard<std::mutex> lock(bufferMutex);
	return fullpose;
}

void SpecificWorker::FullPoseEstimation_setInitialPose(float x, float y, float z, float rx, float ry, float rz)
{
	std::cout << "New initial pose received: " <<x<<" "<<y<<" "<<z<<" "<<rx<<" "<<ry<<" "<<rz<<std::endl;
	initialPose.set(rx, ry, rz, x, y, z);
}
