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

// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include "buffer_types.h"
#include "room_concept.h"
#include "epistemic_planner.h"
#include "viewer_2d.h"
#include "timeseries_plot.h"
#include <atomic>
#include <fps/fps.h>

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

		void FullPoseEstimationPub_newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose);
		void JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data);

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

	private:

		struct Params
		{
			float ROBOT_WIDTH = 0.460;  // m
			float ROBOT_LENGTH = 0.480;  // m
			float ROBOT_HEIGHT = 1.6f;   // m, obstacle cloud ceiling
			float ROBOT_SEMI_WIDTH = ROBOT_WIDTH / 2.f;     // m
			float ROBOT_SEMI_LENGTH = ROBOT_LENGTH / 2.f;    // m
			
			// High LiDAR (HELIOS) — localization + MPPI
			std::string LIDAR_NAME_HIGH = "helios";
			float MAX_LIDAR_HIGH_RANGE = 100;  // m
			int LIDAR_LOW_DECIMATION_FACTOR = 1;
			float 
			
			LIDAR_HIGH_MIN_HEIGHT = 1.2; // m, points below this height in the high lidar will be ignored
			float LIDAR_HIGH_MAX_HEIGHT = 2.2f; // m, points above this height in the high lidar will be ignored
			
			// General
			QRectF GRID_MAX_DIM{-5, -5, 10, 10};
			int MAX_LIDAR_DRAW_POINTS = 500;
			float VIEW_FIT_RADIUS = 6.0f; // m, only nearby lidar points are considered for auto-fit
			bool USE_WEBOTS = false;
			float ODOMETRY_NOISE_FACTOR = 0.1f; // stddev of Gaussian noise added to odometry readings, as a fraction of the measured value (e.g. 0.1 = 10% noise)
			bool PREDICTION_EARLY_EXIT = false; // Skip RFE optimization when predicted pose already has low SDF error

		};
		Params params;

		bool startup_check_flag;

		// Utils
		float yawFromQuaternion(const RoboCompWebots2Robocomp::Quaternion &quat);
		std::chrono::steady_clock::time_point last_joystick_time_;
		FPSCounter fps_counter_;
		
		// Thread-safe buffer for velocity commands (joystick / controller)
		rc::VelocityBuffer velocity_buffer_{20};

		// Measured odometry readings from FullPoseEstimationPub (thread-safe via BufferSync)
		rc::OdometryBuffer odometry_buffer_{20};

		//Viewer
		std::unique_ptr<rc::Viewer2D> viewer_2d_;
		rc::TimeSeriesPlot* ts_plot_sdf_ = nullptr;
		rc::TimeSeriesPlot* ts_plot_fe_  = nullptr;

		// Localization model
		rc::RoomConcept room_concept_;
		bool room_initialized_from_svg_polygon_ = false;
		int rfe_saved_window_size_ = 10;  // saved value when RFE is toggled off

		// Epistemic planner (active inference self-targeting)
		rc::EpistemicPlanner epistemic_planner_;
		bool self_target_active_ = false;
		void navigate_to_target(const std::optional<rc::RoomConcept::UpdateResult>& loc_res,
		                        const std::optional<rc::ObstacleData>& obstacles);

		// Lidar
		// Sync Buffer: slot 0 = GT pose, slot 1 = HELIOS high lidar, slot 2 = BPEARL low lidar
		rc::SensorBuffer lidar_buffer;


		// Lidar Thread
		std::thread read_lidar_th;
		std::atomic<bool> stop_lidar_thread{false};
		std::atomic<bool> pose_saved_{false};
		void read_lidar();
		void save_robot_pose_once();
		std::string pose_file_path() const;
		void update_ui(const std::optional<rc::RoomConcept::UpdateResult>& loc_res,
		               const Eigen::Affine2f& pose_for_draw);
		void update_cpu_label();
		Eigen::Affine2f best_available_pose(
		        const std::optional<rc::RoomConcept::UpdateResult>& loc_res, bool have_loc) const;
		Eigen::Affine2f forward_project_pose(const Eigen::Affine2f& base_pose,
		                                     const std::optional<rc::RoomConcept::UpdateResult>& loc_res,
		                                     std::int64_t lidar_ts);
		// EMA state for smoothing forward projection deltas
		float smooth_dx_ = 0.f, smooth_dy_ = 0.f, smooth_dtheta_ = 0.f;
		bool fwd_proj_initialized_ = false;

		void initialize_room_model_from_svg();
		void save_robot_pose_on_exit() const;

	private slots:
		void slot_robot_moved(QPointF p);
		void slot_robot_rotate(QPointF p);
		void slot_rfe_toggled(bool checked);
		void slot_self_target_toggled(bool checked);

	signals:
		//void customSignal();
};

#endif
