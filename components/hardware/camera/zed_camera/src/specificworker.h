/*
 *    Copyright (C) 2026 by YOUR NAME HERE
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
#define POSE

#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <memory>
#include <Eigen/Dense>
#include <omp.h>
#include "CloudCompressor.h"
#include <shared_mutex>

//#include <FPS1    
// Zero-copy DDS media-plane publisher (RGB + depth), gated by config. FastDDS
// headers stay behind this forward declaration (PIMPL in dds_publisher.cpp).
class ZedDDSPublisher;


// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#include <genericworker.h>

// Basic structure to compare timestamps of a sensor. Determines if a specific sensor data has been updated or not.
struct TimestampHandler {

    // Compare the new timestamp to the last valid one. If it is higher, save it as new reference.
    inline bool isNew(sl::Timestamp& ts_curr, sl::Timestamp& ts_ref) {
        bool new_ = ts_curr > ts_ref;
        if (new_) ts_ref = ts_curr;
        return new_;
    }
    // Specific function for IMUData.
    inline bool isNew(sl::SensorsData::IMUData& imu_data) {
        return isNew(imu_data.timestamp, ts_imu);
    }
    // Specific function for MagnetometerData.
    inline bool isNew(sl::SensorsData::MagnetometerData& mag_data) {
        return isNew(mag_data.timestamp, ts_mag);
    }
    // Specific function for BarometerData.
    inline bool isNew(sl::SensorsData::BarometerData& baro_data) {
        return isNew(baro_data.timestamp, ts_baro);
    }

    sl::Timestamp ts_imu = 0, ts_baro = 0, ts_mag = 0; // Initial values
};

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

        RoboCompCameraRGBDSimple::TRGBD CameraRGBDSimple_getAll(std::string camera);
        RoboCompCameraRGBDSimple::TDepth CameraRGBDSimple_getDepth(std::string camera);
        RoboCompCameraRGBDSimple::TImage CameraRGBDSimple_getImage(std::string camera);
        RoboCompCameraRGBDSimple::TPoints CameraRGBDSimple_getPoints(std::string camera);

        std::string MediaPlaneDDS_getMediaDescriptor();

        RoboCompLidar3D::TColorCloudData Lidar3D_getColorCloudData();
        RoboCompLidar3D::TData Lidar3D_getLidarData(std::string name, float start, float len, int decimationDegreeFactor);
        RoboCompLidar3D::TDataImage Lidar3D_getLidarDataArrayProyectedInImage(std::string name);
        RoboCompLidar3D::TDataCategory Lidar3D_getLidarDataByCategory(RoboCompLidar3D::TCategories categories, Ice::Long timestamp);
        RoboCompLidar3D::TData Lidar3D_getLidarDataProyectedInImage(std::string name);
        RoboCompLidar3D::TData Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor);


        // Create a ZED camera object
        sl::Camera zed;

        // Set configuration parameters
        sl::InitParameters init_parameters;

        sl::CameraParameters cam_params;

        bool simulated = true;
        bool display = false;
        bool publish_rgbd = true;
        bool compute_xyz = true;
        bool compute_color_cloud = false;
        bool publish_dds = false;            // gate the DDS media plane (Config.PublishDDS)
        int  camera_fps = 30;                // Config.FPS — caps the processing/publish rate
        Eigen::Affine3f extrinsic;

        sl::Mat image, depth, point_cloud;
        sl::ERROR_CODE returned_state;

        //----POSE
        sl::PositionalTrackingParameters tracking_parameters;
        sl::Pose zed_pose;
        sl::Transform translation_left_to_center;
        //---- SENSORS
        std::thread sensor_thread;
        sl::SensorsData sensors_data;
        TimestampHandler ts;
        std::atomic<bool> running;
        //----TF cam2robot
        Eigen::Affine3f T_cam2base = Eigen::Affine3f::Identity();
        float wrapToPi(float angle_rad);
        void transformPose(sl::Transform &pose, sl::Transform transform);

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

        void process_pose_data();
        void sensorsLoop();

    private:

        /**
         * \brief Flag indicating whether startup checks are enabled.
         */
        bool startup_check_flag;
        mutable std::shared_mutex color_point_cloud_mutex;
        mutable std::shared_mutex rgbd_mutex;
        std::shared_ptr<RoboCompLidar3D::TColorCloudData> buffer_color_point_cloud;
        std::shared_ptr<RoboCompCameraRGBDSimple::TRGBD> buffer_rgbd;

        // ---- Compute pipeline (see specificworker.cpp) ----
        void regulate_period();                // closed-loop rate control toward Config.FPS
        void step_real();                      // real ZED branch: grab -> build -> publish -> pose
        void step_simulated();                 // simulated branch: proxy pull -> build -> publish
        // Acquire and build the TRGBD frame (+ optional color cloud) for each source.
        // Return the number of valid 3D points, or -1 (simulated only) when no new
        // source frame has arrived since the last call.
        long build_real_rgbd(RoboCompCameraRGBDSimple::TRGBD &rgbd, RoboCompLidar3D::TColorCloudData &colorCloud);
        long build_simulated_rgbd(RoboCompCameraRGBDSimple::TRGBD &rgbd, RoboCompLidar3D::TColorCloudData &colorCloud);
        // Common tail: publish (Ice + DDS) from the local frame (no lock held during
        // serialization), store it into the shared buffers, then optionally display.
        void publish_and_store(RoboCompCameraRGBDSimple::TRGBD &&rgbd, RoboCompLidar3D::TColorCloudData &colorCloud, long num_points);
        void store_rgbd(RoboCompCameraRGBDSimple::TRGBD &&rgbd);
        void store_color_cloud(RoboCompLidar3D::TColorCloudData &colorCloud, long num_points);
        void publish_dds_frames(const RoboCompCameraRGBDSimple::TRGBD &rgbd);
        void show_frames();

        // DDS media-plane publisher (null unless Config.PublishDDS is enabled).
        std::unique_ptr<ZedDDSPublisher> dds_publisher;

    signals:
	    //void customSignal();

};

#endif
