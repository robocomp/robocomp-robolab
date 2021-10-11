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
#include <innermodel/innermodel.h>
#include <qmat/QMatAll>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include <iostream>


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
		RoboCompFullPoseEstimation::FullPoseMatrix FullPoseEstimation_getFullPoseMatrix();
		RoboCompFullPoseEstimation::FullPoseEuler FullPoseEstimation_getFullPoseEuler();
		void FullPoseEstimation_setInitialPose(float x, float y, float z, float rx, float ry, float rz);


public slots:
		void compute();
        int startup_check();
		void initialize(int period);

	private:

        struct PARAMS {
            std::string device_serial;          ///Serial del la camara
            rs2::pipeline pipe;                 ///Pipe de la camara
            Eigen::Affine3f robot_camera;       ///Matriz de camara respecto al robort (config)
            Eigen::Affine3f origen_camera;      ///Matrix de ejes de la camara respecto al origen
            unsigned int mapper_confidence;     //5
            unsigned int tracker_confidence;    //6
            rs2::wheel_odometer* odometer;
            //final pose estimation
            Eigen::Vector3f translation;
            Eigen::Quaternion<float> quatCam;
        };

		bool print_output = false;
		int num_cameras;
        std::map<string, PARAMS> cameras_dict;
        mutable std::mutex bufferMutex;
        bool has_odometry;
        Eigen::Affine3f origen_robot;   //Matrix de ejes de la robot respecto al origen
        Eigen::Vector3f initAngles;

        Eigen::Vector3f quaternion_to_euler_angle(float w, float x, float y, float z);
        float addAng180(float ang, float angAdd);

		std::shared_ptr < InnerModel > innerModel;
		bool startup_check_flag;
};

#endif
