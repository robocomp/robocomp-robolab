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

/**
       \brief
       @author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <qmat/QMatAll>

#pragma push_macro("Q_FOREACH")
#undef Q_FOREACH

#include <iomanip>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

class SpecificWorker : public GenericWorker
{
	Q_OBJECT
	public:
	    SpecificWorker(TuplePrx tprx, bool startup_check);
		~SpecificWorker();
		bool setParams(RoboCompCommonBehavior::ParameterList params);
		rs2::device get_device(const std::string& serial_number);
        RoboCompFullPoseEstimation::FullPoseMatrix FullPoseEstimation_getFullPoseMatrix(){ return RoboCompFullPoseEstimation::FullPoseMatrix();};
		RoboCompFullPoseEstimation::FullPoseEuler FullPoseEstimation_getFullPoseEuler();
		void FullPoseEstimation_setInitialPose(float x, float y, float z, float rx, float ry, float rz);

public slots:
		void compute();
		void initialize(int period);

	private:
		std::string serial;
		bool print_output = false;
		mutable std::mutex bufferMutex;
		RoboCompFullPoseEstimation::FullPoseEuler fullpose;
		// Declare RealSense pipeline, encapsulating the actual device and sensors
		rs2::pipeline pipe;
		// Create a configuration for configuring the pipeline with a non default profile
		rs2::config cfg;
		rs2::context ctx;
		RTMat initialPose;
};

#endif
