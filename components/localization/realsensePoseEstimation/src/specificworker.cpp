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
	innerModelViewer = NULL;
	osgView = new OsgView(this);
	osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));
	osg::Vec3d center(osg::Vec3(0.,0.,-0.));
	osg::Vec3d up(osg::Vec3(0.,1.,0.));
	tb->setHomePosition(eye, center, up, true);
	tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
	osgView->setCameraManipulator(tb);
	
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
//       THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innerModel = std::make_shared<InnerModel>(innermodel_path);
		innerModelViewer = new InnerModelViewer (innerModel, "root", osgView->getRootGroup(), true);
	}
	catch(const std::exception &e) { qFatal("Error reading config params"); }

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	// Add pose stream
	cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
	// Start pipeline with chosen configuration
	pipe.start(cfg);

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

	std::lock_guard<std::mutex> lock(bufferMutex);
		fullpose = {tr.x*1000 + x_offset, 
					tr.y*1000, 
					tr.z*1000 + z_offset, 
					angles.x(), angles.y(), angles.z()};
	
	 
	 innerModel->updateTransformValues("robot", fullpose.x, fullpose.y, fullpose.z, fullpose.rx, fullpose.ry, fullpose.rz );

	// Update innermodelviewer
	 innerModelViewer->update();
	 osgView->frame();
	
}


FullPose SpecificWorker::FullPoseEstimation_getFullPose()
{
	std::lock_guard<std::mutex> lock(bufferMutex);
	return fullpose;
}


