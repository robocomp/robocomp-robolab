/*
 *    Copyright (C) 2024 by YOUR NAME HERE
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
#define USE_OPEN3D false

#if USE_OPEN3D
    #include <open3d/Open3D.h>
#endif

// If you want reduce compute period automaticaly for lack of use
#define HIBERNATION_ENABLED

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
#include <omp.h>
// #include <execution>

typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;
extern robosense::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
extern robosense::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */
class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    public:
        /**
         * \brief Constructor for SpecificWorker.
         * \param configLoader Configuration loader for the component.
         * \param tprx Tuple of proxies required for the component.
         * \param startup_check Indicates whether to perform startup checks.
         */
        SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);
        /**
         * \brief Destructor for SpecificWorker.
         */
        ~SpecificWorker();
	    RoboCompLidar3D::TColorCloudData Lidar3D_getColorCloudData();
        RoboCompLidar3D::TData Lidar3D_getLidarData(std::string name, float start, float len, int decimationDegreeFactor);
        RoboCompLidar3D::TDataImage Lidar3D_getLidarDataArrayProyectedInImage(std::string name);
	    RoboCompLidar3D::TDataCategory Lidar3D_getLidarDataByCategory(RoboCompLidar3D::TCategories categories, Ice::Long timestamp);        
	    RoboCompLidar3D::TData Lidar3D_getLidarDataProyectedInImage(std::string name);
        RoboCompLidar3D::TData Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor);

    public slots:
        /**
         * \brief Initializes the worker one time.
         */
        void initialize();

        /**
         * \brief Main compute loop of the worker.
         */
        void compute();

        /**
         * \brief Handles the emergency state loop.
         */
        void emergency();

        /**
         * \brief Restores the component from an emergency state.
         */
        void restore();

        /**
         * \brief Performs startup checks for the component.
         * \return An integer representing the result of the checks.
         */
        int startup_check();
        /**
         * \brief Flag indicating whether startup checks are enabled.
         */
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
        std::vector<int> compression_params;

        static std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void);
        double remap_angle(double angle);
        int remap_angle_real(int angle);
        static void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg);
        static void exceptionCallback(const robosense::lidar::Error& code);

        // Buffers
        DoubleBuffer<std::pair<RoboCompLidar3D::TData, RoboCompLidar3D::TData>, 
                     std::pair<RoboCompLidar3D::TData, RoboCompLidar3D::TData>> buffer_data;
        DoubleBuffer<std::pair<RoboCompLidar3D::TDataImage, RoboCompLidar3D::TDataImage>,
                     std::pair<RoboCompLidar3D::TDataImage, RoboCompLidar3D::TDataImage>> buffer_array_data;

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
        std::vector<cv::Point2f> fish2equirect(const std::vector<cv::Point2f> &points);
        std::pair<bool, Eigen::Vector3f> transform_filter_point(float x, float y, float z, int intensity);
        std::pair<RoboCompLidar3D::TData, RoboCompLidar3D::TData> processLidarData(const auto &input_points);  // Abbreviated function template
        inline bool isPointOutsideCube(const Eigen::Vector3f point, const Eigen::Vector3f box_min, const Eigen::Vector3f box_max);

        // dowwnsample
        double down_sampling = 0;   //mm free voxel radius

        // Inicialización de rvec
        cv::Mat rvec;
        cv::Mat tvec;
        
    signals:
        //void customSignal();
};

#endif
