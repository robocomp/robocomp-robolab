/*
 *    Copyright (C) 2023 by YOUR NAME HERE
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
#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <fps/fps.h>
#include <doublebuffer/DoubleBuffer.h>


typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;

extern robosense::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
extern robosense::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);

        RoboCompLidar3D::TLidarData Lidar3D_getLidarData(int start, int len);

    public slots:
        void compute();
        int startup_check();
        void initialize(int period);

    private:
        bool startup_check_flag;
        int lidar_model;
        bool simulator;
        int msop_port;
        int difop_port;
        std::string dest_pc_ip_addr;
        robosense::lidar::LidarType lidar_model_list[2] = {
            robosense::lidar::LidarType::RSHELIOS,
            robosense::lidar::LidarType::RSBP
        };

        robosense::lidar::RSDriverParam param;                  ///< Create a parameter object
        robosense::lidar::LidarDriver<PointCloudMsg> driver;               ///< Declare the driver object

        static std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void);
        double remap_angle(double angle);
        int remap_angle_real(int angle);
        static void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg);
        static void exceptionCallback(const robosense::lidar::Error& code);
        float angle_precision = 0.4;
        int lenPoints= 28800;
        int original_fov = 360;
        int points_per_angle = 32;
//        std::PointCloudT<PointXYZI>::VectorT getPointsInRange(const PointCloudT<PointXYZI>::VectorT, float centralAngle, float widthAngle);
        FPSCounter fps;
        DoubleBuffer<PointCloudMsg, RoboCompLidar3D::TLidarData> buffer_data;
};

#endif
