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

    initialize_room_model_from_svg();
    const std::string pose_path = pose_file_path();
    room_concept_.set_seed_pose_file(pose_path);
    std::cout << "Pose seed file path: " << pose_path << std::endl;

    viewer_2d_ = std::make_unique<rc::Viewer2D>(frame, params.GRID_MAX_DIM, true);
    viewer_2d_->show();

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

    // Read lidar
    const auto &[robot_pose_gt_, lidar_data_] = lidar_buffer.read(timestamp);
    if (!lidar_data_.has_value())
    { qWarning() << "No lidar data from lidar_buffer"; return; };

    const auto t1 = std::chrono::high_resolution_clock::now();

    // Draw lidar in room frame using RoomConcept robot pose estimate.
    Eigen::Affine2f pose_for_draw = Eigen::Affine2f::Identity();
    const auto loc_res = room_concept_.get_last_result();
    if (loc_res.has_value() && loc_res->ok)
    {
        pose_for_draw = loc_res->robot_pose;
    }
    else if (room_concept_.is_initialized())
    {
        const auto state = room_concept_.get_current_state();
        pose_for_draw.translation() = Eigen::Vector2f(state[2], state[3]);
        pose_for_draw.linear() = Eigen::Rotation2Df(state[4]).toRotationMatrix();
    }

    // Forward-project the localization pose to the time of the displayed scan.
    // Both lidar_data_->second and loc_res->timestamp_ms are driver-clock
    // acquisition timestamps, so their difference is the real inter-scan lag
    // (includes buffer delay + Adam optimization time).
    Eigen::Affine2f display_pose = pose_for_draw;
    {
        const std::int64_t loc_ts = (loc_res.has_value() && loc_res->ok && loc_res->timestamp_ms > 0)
                                      ? loc_res->timestamp_ms : 0;
        const std::int64_t lag_ms = (loc_ts > 0) ? (lidar_data_->second - loc_ts) : 0;
        const float theta_before = std::atan2(pose_for_draw.linear()(1, 0),
                                              pose_for_draw.linear()(0, 0));
        float theta_after = theta_before;

        if (loc_ts > 0)
        {
            const float dt_s = static_cast<float>(lag_ms) / 1000.0f;
            if (dt_s > 0.001f && dt_s < 1.0f)
            {
                float v_adv = 0.f, v_side = 0.f, v_rot = 0.f;
                bool have_vel = false;
                if (const auto [odom_opt] = odometry_buffer_.read_last(); odom_opt.has_value())
                {
                    v_adv  = odom_opt->adv;
                    v_side = odom_opt->side;
                    v_rot  = odom_opt->rot;
                    have_vel = true;
                }
                else if (const auto [vel_opt] = velocity_buffer_.read_last(); vel_opt.has_value())
                {
                    v_adv  = vel_opt->adv_y;
                    v_side = vel_opt->adv_x;
                    v_rot  = vel_opt->rot;
                    have_vel = true;
                }

                if (have_vel)
                {
                    const float cos_t = std::cos(theta_before);
                    const float sin_t = std::sin(theta_before);
                    const float dx =  (v_side * cos_t - v_adv * sin_t) * dt_s;
                    const float dy =  (v_side * sin_t + v_adv * cos_t) * dt_s;
                    const float dtheta = v_rot * dt_s;

                    display_pose.translation() += Eigen::Vector2f(dx, dy);
                    display_pose.linear() = Eigen::Rotation2Df(theta_before + dtheta).toRotationMatrix();
                    theta_after = theta_before + dtheta;
                }
            }
        }

        // Diagnostic — throttled every 500ms to see correction per frame
        static auto last_diag = std::chrono::steady_clock::now();
        if (std::chrono::steady_clock::now() - last_diag > std::chrono::milliseconds(500))
        {
            last_diag = std::chrono::steady_clock::now();
            std::cout << "[FwdProj] lag_ms=" << lag_ms
                      << " th_before=" << theta_before
                      << " th_after=" << theta_after
                      << " delta=" << (theta_after - theta_before)
                      << " lidar_ts=" << lidar_data_->second
                      << " pose_ts=" << loc_ts << std::endl;
        }
    }
    viewer_2d_->draw_lidar_points(lidar_data_->first, {}, display_pose, params.MAX_LIDAR_DRAW_POINTS);

    // Keep robot glyph synced with the forward-projected display pose.
    if (room_concept_.is_initialized() || (loc_res.has_value() && loc_res->ok))
        viewer_2d_->update_robot(display_pose);

    if (loc_res.has_value() && loc_res->ok)
    {
        if (!room_initialized_from_svg_polygon_)
            viewer_2d_->update_estimated_room_rect(loc_res->state[0], loc_res->state[1], false);
    }

    update_ui(loc_res, pose_for_draw);

    const auto t2 = std::chrono::high_resolution_clock::now();
    const float ms_buf  = std::chrono::duration<float, std::milli>(t1 - t0).count();
    const float ms_draw = std::chrono::duration<float, std::milli>(t2 - t1).count();
    fps_counter_.print("[Compute] buf=" + std::to_string(static_cast<int>(ms_buf))
                       + "ms draw="    + std::to_string(static_cast<int>(ms_draw)) + "ms", 2000);

}

void SpecificWorker::update_ui(const std::optional<rc::RoomConcept::UpdateResult>& loc_res,
                               const Eigen::Affine2f& pose_for_draw)
{
    if (sdfStatusLabel != nullptr)
    {
        if (loc_res.has_value() && loc_res->ok)
            sdfStatusLabel->setText(QString("SDF: %1 m").arg(loc_res->sdf_mse, 0, 'f', 3));
        else
            sdfStatusLabel->setText("SDF: n/a");
    }

    if (poseStatusLabel != nullptr)
    {
        const auto t = pose_for_draw.translation();
        const float theta = std::atan2(pose_for_draw.linear()(1, 0), pose_for_draw.linear()(0, 0));
        poseStatusLabel->setText(
            QString("Pose: x=%1  y=%2  th=%3 rad")
                .arg(t.x(), 0, 'f', 3)
                .arg(t.y(), 0, 'f', 3)
                .arg(theta, 0, 'f', 3));
    }
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
                // data_high = lidar3d_proxy->getLidarDataWithThreshold2d(
                //         params.LIDAR_NAME_HIGH,
                //         params.MAX_LIDAR_HIGH_RANGE * 1000.f,
                //         params.LIDAR_LOW_DECIMATION_FACTOR);
                data_high = lidar3d_proxy->getLidarData("", 0.f, M_PI*2.f, params.LIDAR_LOW_DECIMATION_FACTOR);                    
            }
            catch (const Ice::Exception &e)
            { qWarning() << "[read_lidar] HELIOS failed:" << e.what(); continue;}

            std::vector<Eigen::Vector3f> points_high;
            points_high.reserve(data_high.points.size());
            for (const auto &p : data_high.points)
            {
                const float pmx = p.x / 1000.f;
                const float pmy = p.y / 1000.f;
                const float pmz = p.z / 1000.f;
                if(pmz > params.LIDAR_HIGH_MIN_HEIGHT)
                    points_high.emplace_back(pmx, pmy, pmz);
            }
            const size_t n_pts = points_high.size();
            // Store system-clock timestamp (not driver timestamp) so consumers
            // can compute real wall-clock age of this scan.
            lidar_buffer.put<1>(std::make_pair(std::move(points_high), static_cast<std::int64_t>(data_high.timestamp)), timestamp);

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
			cmd.rot = axis.value;
		else if (axis.name == "advance") // forward is positive Z. Right-hand rule
			cmd.adv_y = axis.value/1000.0f; // from mm/s to m/s
		else if (axis.name == "side")
			cmd.adv_x = 0.0f; // not lateral motion allowed
	}
    cmd.timestamp = std::chrono::high_resolution_clock::now();
    const auto ts = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
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
    odom.adv  = add_noise(pose.adv);    // forward velocity, m/s
    odom.side = add_noise(pose.side);   // lateral velocity, m/s
    odom.rot  = add_noise(pose.rot);    // angular velocity, rad/s
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

