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
#include "specificworker.h"
#include "svg_room_loader.h"

#include <fstream>
#include <unistd.h>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QVBoxLayout>

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif
		

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	// Stop state machine timers to prevent compute/emergency callbacks during teardown
    statemachine.stop();
    
	room_concept_.stop();
    save_robot_pose_once();
	// Stop background threads first
    stop_lidar_thread = true;
    if (read_lidar_th.joinable())
        read_lidar_th.join();
}


void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;
	GenericWorker::initialize();

    QObject::connect(QCoreApplication::instance(), &QCoreApplication::aboutToQuit,
                     this, &SpecificWorker::save_robot_pose_once,
                     Qt::UniqueConnection);

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name")
    params.PREDICTION_EARLY_EXIT = configLoader.get<bool>("RoomConcept.PredictionEarlyExit");
    room_concept_.params.num_iterations          = configLoader.get<int>("RoomConcept.NumIterations");
    room_concept_.params.rfe_window_size         = configLoader.get<int>("RoomConcept.WindowSize");
    room_concept_.params.max_lidar_points        = configLoader.get<int>("RoomConcept.MaxLidarPoints");
    room_concept_.params.rfe_max_lidar_per_old_slot = configLoader.get<int>("RoomConcept.MaxLidarOldSlot");
    room_concept_.params.recovery_loss_threshold    = static_cast<float>(configLoader.get<double>("RoomConcept.RecoveryLossThreshold"));
    room_concept_.params.recovery_consecutive_count = configLoader.get<int>("RoomConcept.RecoveryConsecutiveCount");
    try { room_concept_.params.odom_noise_scale = static_cast<float>(configLoader.get<double>("RoomConcept.OdomNoiseScale")); } catch (...) {}
    try { room_concept_.params.differential_test_enabled = configLoader.get<bool>("RoomConcept.DifferentialTest"); } catch (...) {}

	// Lidar thread is created
    read_lidar_th = std::thread(&SpecificWorker::read_lidar,this);
    qInfo() << __FUNCTION__ << "Started lidar reader";

    // Configure and start localization thread
    rc::RoomConcept::RunContext run_ctx;
    run_ctx.sensor_buffer = &lidar_buffer;
    run_ctx.velocity_buffer = &velocity_buffer_;
    run_ctx.odometry_buffer = &odometry_buffer_;
    room_concept_.set_run_context(run_ctx);
    room_concept_.params.prediction_early_exit = params.PREDICTION_EARLY_EXIT;
    rfe_saved_window_size_ = room_concept_.params.rfe_window_size;

    connect(rfeToggleButton, &QPushButton::toggled, this, &SpecificWorker::slot_rfe_toggled);
    connect(selfTargetButton, &QPushButton::toggled, this, &SpecificWorker::slot_self_target_toggled);

    initialize_room_model_from_svg();
    const std::string pose_path = pose_file_path();
    room_concept_.set_seed_pose_file(pose_path);
    std::cout << "Pose seed file path: " << pose_path << std::endl;

    viewer_2d_ = std::make_unique<rc::Viewer2D>(frame, params.GRID_MAX_DIM, true);
    viewer_2d_->show();

    // Time-series plots inside the bottom frames
    ts_plot_sdf_ = new rc::TimeSeriesPlot(plotFrame);
    ts_plot_sdf_->add_series("sdf_mse", QColor(255, 80, 80), 1.5f, 30);
    ts_plot_sdf_->set_visible_window(30.f);
    auto* sdf_layout = new QVBoxLayout(plotFrame);
    sdf_layout->setContentsMargins(0, 0, 0, 0);
    sdf_layout->addWidget(ts_plot_sdf_);

    ts_plot_fe_ = new rc::TimeSeriesPlot(plotFrame2);
    ts_plot_fe_->add_series("fe_adam", QColor(80, 180, 255), 1.5f, 30);
    ts_plot_fe_->add_series("fe_pred", QColor(255, 160, 50), 1.5f, 30);
    ts_plot_fe_->set_visible_window(30.f);
    auto* fe_layout = new QVBoxLayout(plotFrame2);
    fe_layout->setContentsMargins(0, 0, 0, 0);
    fe_layout->addWidget(ts_plot_fe_);

    viewer_2d_->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0.f, 0.f, QColor("blue"));

    if (room_initialized_from_svg_polygon_)
    {
        const auto room_polygon = rc::SvgRoomLoader::load_polygon_points("beta_layout.svg", "room_contour", false, true);
        if (room_polygon.size() >= 3)
            viewer_2d_->draw_room_polygon(room_polygon, false);
    }
    else
    {
        viewer_2d_->update_estimated_room_rect(params.GRID_MAX_DIM.width(), params.GRID_MAX_DIM.height(), false);
    }

    // Connect mouse interaction signals for manual robot repositioning
    connect(viewer_2d_.get(), &rc::Viewer2D::robot_moved,
            this, &SpecificWorker::slot_robot_moved);
    connect(viewer_2d_.get(), &rc::Viewer2D::robot_rotate,
            this, &SpecificWorker::slot_robot_rotate);

    room_concept_.start();
}


void SpecificWorker::compute()
{
    const auto t0 = std::chrono::high_resolution_clock::now();

    const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    // Read lidar — nothing to do without a scan
    const auto &[robot_pose_gt_, lidar_data_, obstacle_data_] = lidar_buffer.read(timestamp);
    if (!lidar_data_.has_value())
    { qWarning() << "No lidar data from lidar_buffer"; return; }

    const auto t1 = std::chrono::high_resolution_clock::now();

    // Get latest localization result (may be empty before first convergence)
    const auto loc_res = room_concept_.get_last_result();
    const bool have_loc = loc_res.has_value() and loc_res->ok;

    // Build the best available pose: localized → current model state → identity
    const Eigen::Affine2f pose_for_draw = best_available_pose(loc_res, have_loc);

    // Forward-project to compensate localizer-to-display lag, then draw
    const Eigen::Affine2f display_pose = forward_project_pose(pose_for_draw, loc_res, lidar_data_->second);
    
    // Update 2D viewer with new scan and pose
    // Use the localization's own lidar scan when available for spatial consistency
    const Eigen::Affine2f loc_pose = have_loc ? loc_res->robot_pose : display_pose;
    const bool use_loc = have_loc && !loc_res->lidar_scan.empty();
    const std::vector<Eigen::Vector3f>& draw_points =
        use_loc ? loc_res->lidar_scan : lidar_data_->first;

    viewer_2d_->update_frame({
        .lidar_points    = draw_points,
        .display_pose    = display_pose,
        .max_lidar_points = params.MAX_LIDAR_DRAW_POINTS,
        .have_loc        = have_loc,
        .is_initialized  = room_concept_.is_initialized(),
        .has_room_polygon = room_initialized_from_svg_polygon_,
        .room_width      = have_loc ? loc_res->state[0] : 0.f,
        .room_length     = have_loc ? loc_res->state[1] : 0.f,
        .loc_pose        = loc_pose,
        .use_loc_pose    = use_loc,
    });

    // Draw corner detections if available
    if (have_loc && !loc_res->corner_matches.empty())
        viewer_2d_->draw_corners(loc_res->corner_matches, display_pose);
    else
        viewer_2d_->draw_corners({}, display_pose);  // hide all markers

    // Active-inference self-targeting: plan and send commands when enabled
    if (self_target_active_ && have_loc)
        navigate_to_target(loc_res, obstacle_data_);

    update_ui(loc_res, pose_for_draw);

    const auto t2 = std::chrono::high_resolution_clock::now();
    const float ms_buf  = std::chrono::duration<float, std::milli>(t1 - t0).count();
    const float ms_draw = std::chrono::duration<float, std::milli>(t2 - t1).count();
    fps_counter_.print("[Compute] buf=" + std::to_string(static_cast<int>(ms_buf))
                       + "ms draw="    + std::to_string(static_cast<int>(ms_draw)) + "ms", 2000);
}

/////////////////////////////////////////////////////////////////////////////////////
Eigen::Affine2f SpecificWorker::best_available_pose(
        const std::optional<rc::RoomConcept::UpdateResult>& loc_res, bool have_loc) const
{
    if (have_loc)
        return loc_res->robot_pose;
    if (room_concept_.is_initialized())
    {
        const auto s = room_concept_.get_current_state();
        Eigen::Affine2f p = Eigen::Affine2f::Identity();
        p.translation() = Eigen::Vector2f(s[2], s[3]);
        p.linear() = Eigen::Rotation2Df(s[4]).toRotationMatrix();
        return p;
    }
    return Eigen::Affine2f(Eigen::Affine2f::Identity());
}

/////////////////////////////////////////////////////////////////////////////////////
Eigen::Affine2f SpecificWorker::forward_project_pose(
        const Eigen::Affine2f& base_pose,
        const std::optional<rc::RoomConcept::UpdateResult>& loc_res,
        std::int64_t lidar_ts)
{
    constexpr float kAlpha = 0.35f;   // EMA smoothing factor (0 = full smooth, 1 = no smooth)

    const std::int64_t loc_ts = (loc_res.has_value() && loc_res->ok && loc_res->timestamp_ms > 0)
                                  ? loc_res->timestamp_ms : 0;
    const std::int64_t lag_ms = (loc_ts > 0) ? (lidar_ts - loc_ts) : 0;
    const float theta_base = std::atan2(base_pose.linear()(1, 0), base_pose.linear()(0, 0));

    // Read latest velocity in robot frame
    float v_adv = 0.f, v_side = 0.f, v_rot = 0.f;
    if (loc_ts > 0)
    {
        if (const auto [odom_opt] = odometry_buffer_.read_last(); odom_opt.has_value())
        {
            v_adv  = odom_opt->adv;
            v_side = odom_opt->side;
            v_rot  = odom_opt->rot;
        }
        else if (const auto [vel_opt] = velocity_buffer_.read_last(); vel_opt.has_value())
        {
            v_adv  = vel_opt->adv_y;
            v_side = vel_opt->adv_x;
            v_rot  = vel_opt->rot;
        }
    }

    // EMA smoothing on velocities (robot frame), not on deltas
    if (!fwd_proj_initialized_)
    {
        smooth_v_adv_  = v_adv;
        smooth_v_side_ = v_side;
        smooth_v_rot_  = v_rot;
        fwd_proj_initialized_ = true;
    }
    else
    {
        smooth_v_adv_  = kAlpha * v_adv  + (1.f - kAlpha) * smooth_v_adv_;
        smooth_v_side_ = kAlpha * v_side + (1.f - kAlpha) * smooth_v_side_;
        smooth_v_rot_  = kAlpha * v_rot  + (1.f - kAlpha) * smooth_v_rot_;
    }

    // Recompute delta from smoothed velocity × current lag
    const float dt_s = static_cast<float>(lag_ms) / 1000.0f;
    const float clamped_dt = (dt_s > 0.001f && dt_s < 1.0f) ? dt_s : 0.f;
    const float cos_t = std::cos(theta_base);
    const float sin_t = std::sin(theta_base);
    const float dx     = (smooth_v_side_ * cos_t - smooth_v_adv_ * sin_t) * clamped_dt;
    const float dy     = (smooth_v_side_ * sin_t + smooth_v_adv_ * cos_t) * clamped_dt;
    const float dtheta = smooth_v_rot_ * clamped_dt;

    Eigen::Affine2f result = base_pose;
    result.translation() += Eigen::Vector2f(dx, dy);
    result.linear() = Eigen::Rotation2Df(theta_base + dtheta).toRotationMatrix();

    return result;
}

/////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::update_ui(const std::optional<rc::RoomConcept::UpdateResult>& loc_res,
                               const Eigen::Affine2f& pose_for_draw)
{
    const bool have_loc = loc_res.has_value() && loc_res->ok;

    // SDF / Free-Energy status
    if (have_loc)
    {
        const bool used_adam = loc_res->iterations_used > 0;
        const QString tag = used_adam ? "[Adam]" : "[Pred]";
        sdfStatusLabel->setText(QString("%1 SDF: %2 m  FE: %3")
                                    .arg(tag)
                                    .arg(loc_res->sdf_mse, 0, 'f', 3)
                                    .arg(loc_res->final_loss, 0, 'f', 3));
        if (ts_plot_sdf_) ts_plot_sdf_->add_point("sdf_mse", loc_res->sdf_mse);
        if (ts_plot_fe_)
        {
            if (used_adam)
                ts_plot_fe_->add_point("fe_adam", loc_res->final_loss);
            else
                ts_plot_fe_->add_point("fe_pred", loc_res->final_loss);
        }
    }
    else
        sdfStatusLabel->setText("SDF: n/a");

    // Pose status
    const auto t = pose_for_draw.translation();
    const float theta = std::atan2(pose_for_draw.linear()(1, 0), pose_for_draw.linear()(0, 0));
    poseStatusLabel->setText(
        QString("Pose: x=%1  y=%2  th=%3 rad")
            .arg(t.x(), 0, 'f', 3)
            .arg(t.y(), 0, 'f', 3)
            .arg(theta, 0, 'f', 3));

    // CPU usage — update ~1 Hz
    update_cpu_label();

    // Velocity display — show latest command or odometry
    float vx = 0.f, vy = 0.f, vr = 0.f;
    if (const auto [vel_opt] = velocity_buffer_.read_last(); vel_opt.has_value())
    { vx = vel_opt->adv_x; vy = vel_opt->adv_y; vr = vel_opt->rot; }
    else if (const auto [odom_opt] = odometry_buffer_.read_last(); odom_opt.has_value())
    { vx = odom_opt->side; vy = odom_opt->adv; vr = odom_opt->rot; }
    velocityLabel->setText(
        QString("Vel: vx=%1  vy=%2  rot=%3")
            .arg(vx, 0, 'f', 3).arg(vy, 0, 'f', 3).arg(vr, 0, 'f', 3));
}

/////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::update_cpu_label()
{
    static std::int64_t prev_utime = 0, prev_stime = 0;
    static std::chrono::steady_clock::time_point prev_wall = std::chrono::steady_clock::now();
    static bool first_cpu = true;

    auto now = std::chrono::steady_clock::now();
    const float wall_s = std::chrono::duration<float>(now - prev_wall).count();
    if (wall_s < 1.0f) return;

    std::ifstream stat_file("/proc/self/stat");
    if (!stat_file.is_open()) return;

    std::string ignore;
    std::int64_t utime = 0, stime = 0;
    stat_file >> ignore >> ignore >> ignore;          // pid, comm, state
    for (int i = 4; i <= 13; ++i) stat_file >> ignore;
    stat_file >> utime >> stime;

    if (!first_cpu)
    {
        const long ticks_hz = sysconf(_SC_CLK_TCK);
        const float cpu_sec = static_cast<float>((utime - prev_utime) + (stime - prev_stime))
                              / static_cast<float>(ticks_hz);
        const float cpu_pct = (cpu_sec / wall_s) * 100.f;
        cpuLabel->setText(QString("CPU: %1%").arg(cpu_pct, 0, 'f', 1));
    }
    first_cpu = false;
    prev_utime = utime;
    prev_stime = stime;
    prev_wall = now;
}

/////////////////////////////////////////////////////////////////////////
void SpecificWorker::slot_robot_moved(QPointF p)
{
    // Shift+Left click: reposition robot at the clicked scene point, keeping current theta
    const auto state = room_concept_.get_current_state();
    const float theta = state[4];
    room_concept_.push_command(rc::RoomConcept::CmdSetPose{
        static_cast<float>(p.x()), static_cast<float>(p.y()), theta});
    qInfo() << "[UI] Robot repositioned to" << p.x() << p.y() << "theta=" << theta;
}

void SpecificWorker::slot_robot_rotate(QPointF p)
{
    // Ctrl+Left click/drag: rotate robot to face the cursor position
    const auto state = room_concept_.get_current_state();
    const float rx = state[2];
    const float ry = state[3];
    const float dx = static_cast<float>(p.x()) - rx;
    const float dy = static_cast<float>(p.y()) - ry;
    const float new_theta = std::atan2(dy, dx);
    room_concept_.push_command(rc::RoomConcept::CmdSetPose{rx, ry, new_theta});
}

void SpecificWorker::slot_rfe_toggled(bool checked)
{
    if (checked)
    {
        room_concept_.params.rfe_window_size = rfe_saved_window_size_;
        rfeToggleButton->setText("RFE ON");
        qInfo() << "[UI] RFE enabled, window size =" << rfe_saved_window_size_;
    }
    else
    {
        rfe_saved_window_size_ = room_concept_.params.rfe_window_size;
        room_concept_.params.rfe_window_size = 1;
        rfeToggleButton->setText("RFE OFF");
        qInfo() << "[UI] RFE disabled (window size = 1)";
    }
}

void SpecificWorker::slot_self_target_toggled(bool checked)
{
    self_target_active_ = checked;
    if (checked)
    {
        selfTargetButton->setText("Self-Target ON");
        qInfo() << "[UI] Self-targeting enabled";
    }
    else
    {
        selfTargetButton->setText("Self-Target OFF");
        // Stop the robot, clear planner target, hide target marker
        try { omnirobot_proxy->setSpeedBase(0.f, 0.f, 0.f); }
        catch (const Ice::Exception& e) { qWarning() << "[SelfTarget] stop failed:" << e.what(); }
        epistemic_controller_.clear_target();
        viewer_2d_->update_target_marker(0.f, 0.f, false);
        qInfo() << "[UI] Self-targeting disabled, robot stopped";
    }
}

/////////////////////////////////////////////////////////////////////////
void SpecificWorker::navigate_to_target(const std::optional<rc::RoomConcept::UpdateResult>& loc_res,
                                         const std::optional<rc::ObstacleData>& obstacles)
{
    // Update planner with current robot state
    const Eigen::Matrix3f cov = loc_res->covariance;

    // Room bounds from localized state: state = [half_w, half_h, x, y, theta]
    const float hw = loc_res->state[0];
    const float hh = loc_res->state[1];
    epistemic_controller_.set_room_bounds({-hw, -hh}, {hw, hh});
    epistemic_controller_.set_robot_state(loc_res->robot_pose, cov);

    // Pass room polygon for FIM-based corner visibility scoring
    const auto& poly = room_concept_.polygon_vertices();
    if (!poly.empty())
        epistemic_controller_.set_room_polygon(poly);

    // Pass floor-projected obstacle cloud
    if (obstacles.has_value())
        epistemic_controller_.set_lidar_obstacles(*obstacles);

    // Plan and execute
    auto result = epistemic_controller_.plan();
    if (!result || !result->valid)
        return;

    // Diagnostic: covariance diag and target type
    const Eigen::Vector2f robot_p{loc_res->robot_pose.translation()};
    const float dist_to_tgt = (result->target.position - robot_p).norm();
    const Eigen::Vector2f tb_diag = loc_res->robot_pose.inverse() * result->target.position;
    const float angle_err_diag = std::atan2(-tb_diag.x(), tb_diag.y());

    qInfo().nospace()
        << "[SelfTarget] d=" << QString::number(dist_to_tgt, 'f', 2)
        << " ae=" << QString::number(angle_err_diag, 'f', 2)
        << " cmd:(" << QString::number(result->command.adv_x, 'f', 3)
        << "," << QString::number(result->command.adv_y, 'f', 3)
        << "," << QString::number(result->command.rot, 'f', 2)
        << ") efe=" << QString::number(result->best_policy.efe, 'f', 1)
        << " epist=" << QString::number(result->best_policy.epistemic_value, 'f', 2)
        << " pragm=" << QString::number(result->best_policy.pragmatic_value, 'f', 2)
        << " obs=" << QString::number(result->best_policy.obstacle_value, 'f', 1)
        << " bnd=" << QString::number(result->best_policy.boundary_value, 'f', 1)
        << " tgt:(" << QString::number(result->target.position.x(), 'f', 1)
        << "," << QString::number(result->target.position.y(), 'f', 1)
        << ") rot_ip:" << result->target.rotate_in_place;

    // Show target on the 2D canvas
    const auto& tgt = result->target.position;
    viewer_2d_->update_target_marker(tgt.x(), tgt.y(), true);

    // Draw the selected trajectory arc (and candidates)
    if (!result->best_policy.predicted_states.empty())
    {
        std::vector<std::vector<Eigen::Vector3f>> cand_states;
        cand_states.reserve(result->all_policies.size());
        for (const auto& p : result->all_policies)
            if (!p.predicted_states.empty())
                cand_states.push_back(p.predicted_states);
        viewer_2d_->draw_all_trajectories(cand_states, result->best_policy.predicted_states);
    }

    const auto& cmd = result->command;
    try
    {
        // Planner and OmniRobot API both CCW+ after Webots proto fix
        omnirobot_proxy->setSpeedBase(cmd.adv_x * 1000.f, cmd.adv_y * 1000.f, cmd.rot);
    }
    catch (const Ice::Exception& e)
    {
        qWarning() << "[SelfTarget] setSpeedBase failed:" << e.what();
    }

    // Buffer the command velocity so forward projection and SDF prediction can use it.
    rc::VelocityCommand vel_cmd(cmd.adv_x, cmd.adv_y, cmd.rot);
    vel_cmd.recv_ts_ms = static_cast<std::int64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    const auto ts = static_cast<std::uint64_t>(vel_cmd.recv_ts_ms);
    velocity_buffer_.put<0>(std::move(vel_cmd), ts);
}

/////////////////////////////////////////////////////////////////////////
void SpecificWorker::read_lidar()
{
    FPSCounter lidar_fps;
    auto wait_period = std::chrono::milliseconds (getPeriod("Compute"));
    while(!stop_lidar_thread)
    {
        const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        try
        {
            // Get robot GT pose from Webots (only for debug/stats, not used by algorithm)
            if (params.USE_WEBOTS)
            {
                const auto &[position, orientation] = webots2robocomp_proxy->getObjectPose("shadow");
                Eigen::Affine2f eig_pose;
                eig_pose.translation() = Eigen::Vector2f(-position.y/1000.f, position.x/1000.f);
                eig_pose.linear() = Eigen::Rotation2Df(yawFromQuaternion(orientation)).toRotationMatrix();
                lidar_buffer.put<0>(std::move(eig_pose), timestamp);
            }

            //const float body_offset_sq = params.ROBOT_SEMI_WIDTH * params.ROBOT_SEMI_WIDTH;

            // HELIOS ----
            RoboCompLidar3D::TData data_high;
            try
            {
                data_high = lidar3d_proxy->getLidarData("", 0.f, M_PI*2.f, params.LIDAR_LOW_DECIMATION_FACTOR);                    
            }
            catch (const Ice::Exception &e)
            { qWarning() << "[read_lidar] HELIOS failed:" << e.what(); continue;}

            std::vector<Eigen::Vector3f> points_high;
            std::vector<Eigen::Vector2f> obstacle_points;
            points_high.reserve(data_high.points.size());
            obstacle_points.reserve(data_high.points.size());
            for (const auto &p : data_high.points)
            {
                const float pmx = p.x / 1000.f;
                const float pmy = p.y / 1000.f;
                const float pmz = p.z / 1000.f;
                if(pmz > params.LIDAR_HIGH_MIN_HEIGHT)
                    points_high.emplace_back(pmx, pmy, pmz);
                if(pmz > 0.1f && pmz < params.ROBOT_HEIGHT)
                    obstacle_points.emplace_back(pmx, pmy);
            }
            const size_t n_pts = points_high.size();
            // Store system-clock timestamp (not driver timestamp) so consumers
            // can compute real wall-clock age of this scan.
            lidar_buffer.put<1>(std::make_pair(std::move(points_high), static_cast<std::int64_t>(data_high.timestamp)), timestamp);
            lidar_buffer.put<2>(std::move(obstacle_points), timestamp);

            // Adjust period with hysteresis
            if (wait_period > std::chrono::milliseconds((long) data_high.period + 2)) --wait_period;
            else if (wait_period < std::chrono::milliseconds((long) data_high.period - 2)) ++wait_period;
            lidar_fps.print("[LidarThread] pts=" + std::to_string(n_pts), 2000);
            std::this_thread::sleep_for(wait_period);
        }
        catch (const Ice::Exception &e)
        { qWarning() << "Error reading from Lidar3D or robot pose:" << e.what(); }
    }
} // Thread to read the lidar

float SpecificWorker::yawFromQuaternion(const RoboCompWebots2Robocomp::Quaternion &quat)
{
    double w = quat.w;
    double x = quat.x;
    double y = quat.y;
    double z = quat.z;
    const auto norm = std::sqrt(w*w + x*x + y*y + z*z);
    w /= norm; x /= norm; y /= norm; z /= norm;
    return static_cast<float>(std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)));
}

void SpecificWorker::initialize_room_model_from_svg()
{
    // Configure room geometry source. RoomConcept thread will bootstrap pose from lidar/saved seed.
    const auto room_polygon = rc::SvgRoomLoader::load_polygon_points("beta_layout.svg", "room_contour", false, true);
    if (room_polygon.size() >= 3)
    {
        room_concept_.configure_room_from_polygon(room_polygon);
        room_initialized_from_svg_polygon_ = true;
        qInfo() << "Configured RoomConcept with mirrored SVG polygon from beta_layout.svg"
                << "vertices:" << room_polygon.size();
        return;
    }

    room_concept_.configure_room_from_rect(params.GRID_MAX_DIM.width(), params.GRID_MAX_DIM.height());
    room_initialized_from_svg_polygon_ = false;
    qWarning() << "SVG polygon not loaded. Configured rectangular fallback room model.";
}

void SpecificWorker::save_robot_pose_on_exit() const
{
    Eigen::Vector3f pose = Eigen::Vector3f::Zero();
    if (const auto loc = room_concept_.get_last_result(); loc.has_value() && loc->ok)
    {
        pose[0] = loc->state[2];
        pose[1] = loc->state[3];
        pose[2] = loc->state[4];
    }
    else if (room_concept_.is_initialized())
    {
        const auto state = room_concept_.get_current_state();
        pose[0] = state[2];
        pose[1] = state[3];
        pose[2] = state[4];
    }
    else
    {
        return;
    }

    const QString qpath = QString::fromStdString(pose_file_path());
    const QFileInfo fi(qpath);
    QDir().mkpath(fi.absolutePath());

    std::ofstream out(qpath.toStdString(), std::ios::trunc);
    if (!out.is_open())
    {
        qWarning() << "save_robot_pose_on_exit(): cannot open pose file at" << qpath;
        return;
    }
    out << pose[0] << ' ' << pose[1] << ' ' << pose[2] << '\n';
    std::cout << "Saved robot pose to " << qpath.toStdString() << ": "
              << pose[0] << " " << pose[1] << " " << pose[2] << std::endl;
}

void SpecificWorker::save_robot_pose_once()
{
    if (pose_saved_.exchange(true))
        return;
    save_robot_pose_on_exit();
}

std::string SpecificWorker::pose_file_path() const
{
    auto find_etc_upwards = [](const QString& start) -> QString
    {
        QDir dir(start);
        for (int depth = 0; depth < 8; ++depth)
        {
            const QString etc_dir = dir.absoluteFilePath("etc");
            if (QDir(etc_dir).exists())
                return etc_dir;
            if (!dir.cdUp())
                break;
        }
        return {};
    };

    const QString from_app = find_etc_upwards(QCoreApplication::applicationDirPath());
    if (!from_app.isEmpty())
        return (from_app + "/last_robot_pose.txt").toStdString();

    const QString from_cwd = find_etc_upwards(QDir::currentPath());
    if (!from_cwd.isEmpty())
        return (from_cwd + "/last_robot_pose.txt").toStdString();

    const QString fallback = QDir(QCoreApplication::applicationDirPath() + "/../etc").absolutePath();
    return (fallback + "/last_robot_pose.txt").toStdString();
}
////////////////////////////////////////////////////////////////////////
void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}


//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// INTERFACES CALLBACKS
//////////////////////////////////////////////////////////////////////////////

void SpecificWorker::JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data)
{
	rc::VelocityCommand cmd;
	for (const auto &axis: data.axes)
	{
		if (axis.name == "rotate")
		{
			cmd.rot = axis.value;    // joystick is CCW+ after proto fix; store directly
		}
		else if (axis.name == "advance") // forward is positive Z. Right-hand rule
			cmd.adv_y = axis.value/1000.0f; // from mm/s to m/s
		else if (axis.name == "side")
			cmd.adv_x = 0.0f; // not lateral motion allowed
	}
    cmd.timestamp = std::chrono::high_resolution_clock::now();
    cmd.recv_ts_ms = static_cast<std::int64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    const auto ts = static_cast<std::uint64_t>(cmd.recv_ts_ms);
	velocity_buffer_.put<0>(std::move(cmd), ts);
}

//SUBSCRIPTION to newFullPose method from FullPoseEstimationPub interface
void SpecificWorker::FullPoseEstimationPub_newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose)
{
    // Add configurable Gaussian noise to simulate realistic odometry uncertainty
    static std::mt19937 gen{std::random_device{}()};
    const float nf = params.ODOMETRY_NOISE_FACTOR;

    auto add_noise = [&](float value) -> float
    {
        if (nf <= 0.f || value == 0.f) return value;
        std::normal_distribution<float> dist(0.f, std::abs(value) * nf);
        return value + dist(gen);
    };

    rc::OdometryReading odom;
    odom.adv  = add_noise(-pose.adv);    // forward velocity, m/s
    odom.side = add_noise(pose.side);   // lateral velocity, m/s
    odom.rot  = add_noise(pose.rot);   // FPE is CCW+ after proto fix; store directly
    odom.source_ts_ms = pose.timestamp; // sensor-side epoch-ms
    odom.recv_ts_ms   = static_cast<std::int64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    odom.timestamp = std::chrono::high_resolution_clock::time_point(
        std::chrono::milliseconds(pose.timestamp));
    const auto ts = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    odometry_buffer_.put<0>(std::move(odom), ts);
}


/**************************************/
// From the RoboCompLidar3D you can call this methods:
// RoboCompLidar3D::TColorCloudData this->lidar3d_proxy->getColorCloudData()
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarData(string name, float start, float len, int decimationDegreeFactor)
// RoboCompLidar3D::TDataImage this->lidar3d_proxy->getLidarDataArrayProyectedInImage(string name)
// RoboCompLidar3D::TDataCategory this->lidar3d_proxy->getLidarDataByCategory(TCategories categories, long timestamp)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataProyectedInImage(string name)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataWithThreshold2d(string name, float distance, int decimationDegreeFactor)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData

