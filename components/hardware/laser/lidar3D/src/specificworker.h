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
#include <atomic>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv4/opencv2/opencv.hpp>

#include "cppitertools/slice.hpp"
#include "cppitertools/zip.hpp"
#include "math.h"

#include <chrono>
#include <thread>

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
            RoboCompLidar3D::TData Lidar3D_getLidarData(std::string name, float start, float len, int decimationDegreeFactor);
            RoboCompLidar3D::TDataImage Lidar3D_getLidarDataArrayProyectedInImage(std::string name);
            RoboCompLidar3D::TData Lidar3D_getLidarDataProyectedInImage(std::string name);
            RoboCompLidar3D::TData Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor);

        public slots:
            void compute();
            int startup_check();
            void initialize(int period);
            void self_adjust_period(int new_period);

private:
            bool startup_check_flag;
            std::atomic_bool ready_to_go = false;
            int lidar_model;
            int msop_port;
            int difop_port;
            std::string dest_pc_ip_addr;
            robosense::lidar::LidarType lidar_model_list[2] =
            {
                    robosense::lidar::LidarType::RSHELIOS,
                    robosense::lidar::LidarType::RSBP
            };

            robosense::lidar::RSDriverParam param;                           ///< Create a parameter object
            robosense::lidar::LidarDriver<PointCloudMsg> driver;             ///< Declare the driver object
            vector<int> compression_params;

            static std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void);
            double remap_angle(double angle);
            int remap_angle_real(int angle);
            static void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg);
            static void exceptionCallback(const robosense::lidar::Error& code);

            // Buffers
            DoubleBuffer<RoboCompLidar3D::TData, RoboCompLidar3D::TData> buffer_data;
            DoubleBuffer<RoboCompLidar3D::TDataImage,RoboCompLidar3D::TDataImage> buffer_array_data;

            //Extrinsic
            Eigen::Affine3f robot_lidar;
            Eigen::Vector3f box_min;
            Eigen::Vector3f box_max;
            float floor_line;
            float top_line;

            //Image
            int img_width = 1200, img_height = 600;

            //Dst image
            int dst_width = 1920, dst_height = 960;

            // SIMULATOR
            bool simulator = false;

            // FPS
            FPSCounter fps;
            std::atomic<std::chrono::high_resolution_clock::time_point> last_read;
            int MAX_INACTIVE_TIME = 5;  // secs after which the component is paused. It reactivates with a new reset

            RoboCompLidar3D::TDataImage lidar2cam(const RoboCompLidar3D::TData &lidar_data);
            std::vector<cv::Point2f> fish2equirect(const vector<cv::Point2f> &points);
            std::optional<RoboCompLidar3D::TPoint> transform_filter_point(float x, float y, float z, int intensity);
            RoboCompLidar3D::TData processLidarData(const auto &input_points);  // Abbreviated function template
            inline bool isPointOutsideCube(const Eigen::Vector3f point, const Eigen::Vector3f box_min, const Eigen::Vector3f box_max);
};

#endif
