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

#include <memory>

typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;
extern robosense::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
extern robosense::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

// Zero-copy DDS media-plane publisher (LidarFrame), gated by config. FastDDS headers
// stay behind this forward declaration (PIMPL in dds_publisher.cpp).
class LidarDDSPublisher;

// Robot-body self-filter (Embree). Embree headers stay behind this forward declaration.
class MeshFilter;

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

        std::string MediaPlaneDDS_getMediaDescriptor();

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
        // ── Source self-synchronisation (SIMULATED branch only) ──────────────────────────────
        // The bridge serves Lidar3D as a PULL servant: there is no arrival to react to, so this
        // component polls. A fixed poll period that is not a divisor of the source period both
        // downsamples the source and carries its phase error into the published stamp. Measured
        // 2026-09-10 with Period.Compute = 50 on a 32 ms source: stamp deltas came out BIMODAL at
        // 32/64 ms (the downsampling signature), 20 Hz published from a ~31 Hz source — about 35%
        // of the scans discarded — and the frames that did survive reached the media plane a mean
        // 35 ms old, with the p10-p90 spread equal to one poll period.
        //
        // Cure (same one read_lidar_thread uses in robot_concept): measure the SOURCE period from
        // consecutive source stamps — never from our own loop timing, which is circular and
        // death-spirals the rate down — poll at ~2x that, and drop a repeated stamp before it costs
        // a mesh filter, a projection and a publish.
        // ★ The period estimate is a decaying MINIMUM, not an average, and that is not a detail:
        // a MISSED source frame can only ever make a stamp delta LARGER (an integer multiple of the
        // true period), never smaller. Averaging those deltas therefore feeds a positive loop —
        // miss a frame, over-estimate the period, poll slower, miss more. Measured 2026-09-10 with an
        // EMA here: the poll oscillated 32-35 ms and logged a setPeriod on nearly every cycle, and
        // the published rate stalled below the source. The minimum is immune to that by construction;
        // the slow upward relaxation is what lets it still follow a source that genuinely slows down.
        std::uint64_t last_src_stamp_ms_ = 0;   // last source stamp accepted (dedup key)
        double        src_period_ms_ = -1.0;    // decaying MIN of source stamp deltas, ms; <0 = unknown
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
        DoubleBuffer<RoboCompLidar3D::TData, RoboCompLidar3D::TData> buffer_data;
        DoubleBuffer<RoboCompLidar3D::TDataImage, RoboCompLidar3D::TDataImage> buffer_array_data;

        //Extrinsic
        Eigen::Affine3f robot_lidar;

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
        RoboCompLidar3D::TData processLidarData(const auto &input_points);  // Abbreviated function template
        inline bool isPointOutsideCube(const Eigen::Vector3f point, const Eigen::Vector3f box_min, const Eigen::Vector3f box_max);

        // Inicialización de rvec
        cv::Mat rvec;
        cv::Mat tvec;

        // Optional zero-copy DDS lidar media plane (null unless PublishDDS is enabled).
        bool publish_dds = false;
        std::unique_ptr<LidarDDSPublisher> dds_publisher;

    // ── PER-RING VERTICAL ANGLES, measured once from a complete sweep ────────────────────────
    // The descriptor advertises the beam geometry, and the per-ring table is the one field that
    // cannot honestly be written by hand: a real H32F70's channels are NOT uniformly spaced (the
    // device reports its own table in the difop packet), and for the simulated unit the placement
    // of N layers across the span is ambiguous from outside — 70/31 and 70/32 differ by 3%, and
    // residual_concept's refuted-band width is proportional to it.
    // Measured on the DEVICE-frame cloud, upstream of the mesh self-filter: processLidarData()
    // deliberately does not apply the mount, so `theta` here is the beam's own angle. Doing it
    // after the filter would bin only the returns that survived a z- and radius-banded cut, which
    // preferentially removes the low rings — the ones that matter most for the near field.
    // Published ONLY when exactly `model_rings_` distinct angles are seen: a partial table
    // advertised as a full one is worse than absent, and absent already means "unknown".
    int  model_rings_ = 0;             // from SensorModel.Rings; 0 = no model configured
    bool ring_elev_published_ = false; // one-shot latch
    // ★ ACCUMULATED across sweeps, because ONE sweep does not contain all the rings. A ring is only
    // visible in a frame where at least one of its beams came back, and the extreme rings of a fan
    // aimed into open space may return nothing for many seconds — measured 2026-09-10 on the live
    // helios: 26 of 32 rings present, elevation span 56 deg of the declared 70, the missing ones all
    // at the bottom. The old single-frame test therefore asked for something the data may never hold
    // at one instant, so it never published and re-announced that failure on every frame. Rings are a
    // FIXED property of the device: seeing them at different times is as good as seeing them at once.
    std::vector<std::pair<double,int>> ring_elev_acc_;   // (running mean elevation deg, samples), ascending
    int ring_elev_reported_ = -1;      // last cluster count logged; -1 = nothing said yet
        std::vector<float> lidar_xyz;   // reusable interleaved x,y,z (metres) publish buffer

        // Optional robot-body self-filter (null unless MeshFilter.enabled). Removes
        // points hitting the static robot mesh; runs once in compute() before publish.
        bool mesh_filter_enabled = false;
        std::unique_ptr<MeshFilter> mesh_filter;

    signals:
        //void customSignal();
};

#endif
