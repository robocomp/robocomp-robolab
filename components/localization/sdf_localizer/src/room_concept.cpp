#include "room_concept.h"
#include "pointcloud_center_estimator.h"

#include <fstream>
#include <iostream>
#include <limits>
#include <QDebug>

namespace rc
{
    // =====================================================================
    //  Threading: start / stop / run / get_last_result / push_command
    // =====================================================================

    // Static initialization: limit PyTorch threads to avoid CPU overload
    static bool torch_threads_initialized = []() {
        torch::set_num_threads(5);
        torch::set_num_interop_threads(2);
        return true;
    }();

    RoomConcept::~RoomConcept()
    {
        stop();
    }

    void RoomConcept::start()
    {
        if (loc_running_.load()) return;
        stop_requested_ = false;
        loc_thread_ = std::thread(&RoomConcept::run, this);
    }

    void RoomConcept::stop()
    {
        stop_requested_ = true;
        if (loc_thread_.joinable())
            loc_thread_.join();
    }

    std::optional<RoomConcept::UpdateResult> RoomConcept::get_last_result() const
    {
        std::lock_guard lock(result_mutex_);
        return last_result_;
    }

    Eigen::Matrix<float,5,1> RoomConcept::get_loc_state() const
    {
        std::lock_guard lock(result_mutex_);
        if (last_result_.has_value() && last_result_->ok)
            return last_result_->state;
        return Eigen::Matrix<float,5,1>::Zero();
    }

    void RoomConcept::push_command(Command cmd)
    {
        std::lock_guard lock(cmd_mutex_);
        pending_commands_.push_back(std::move(cmd));
    }

    void RoomConcept::configure_room_from_polygon(const std::vector<Eigen::Vector2f>& polygon_vertices)
    {
        init_use_polygon_ = true;
        init_polygon_vertices_ = polygon_vertices;
    }

    void RoomConcept::configure_room_from_rect(float width, float length)
    {
        init_use_polygon_ = false;
        init_room_width_ = width;
        init_room_length_ = length;
    }

    void RoomConcept::set_seed_pose_file(const std::string& pose_file_path)
    {
        seed_pose_file_path_ = pose_file_path;
    }

    float RoomConcept::estimate_orientation_from_points(const std::vector<Eigen::Vector3f>& pts) const
    {
        if (pts.size() < 2)
            return 0.f;

        Eigen::Vector2f mean = Eigen::Vector2f::Zero();
        for (const auto& p : pts)
            mean += p.head<2>();
        mean /= static_cast<float>(pts.size());

        Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
        for (const auto& p : pts)
        {
            const Eigen::Vector2f d = p.head<2>() - mean;
            cov += d * d.transpose();
        }
        cov /= std::max<std::size_t>(1, pts.size() - 1);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es(cov);
        if (es.info() != Eigen::Success)
            return 0.f;

        const Eigen::Vector2f principal = es.eigenvectors().col(1);
        return std::atan2(principal.y(), principal.x());
    }

    void RoomConcept::resolve_initial_yaw_ambiguity(const std::vector<Eigen::Vector3f>& lidar_points, float prior_phi)
    {
        constexpr float kYawTieTolerance = 0.02f;
        if (model_ == nullptr || lidar_points.empty())
            return;

        auto wrap_angle = [](float a)
        {
            while (a > M_PI) a -= 2.f * M_PI;
            while (a < -M_PI) a += 2.f * M_PI;
            return a;
        };
        auto ang_dist = [&](float a, float b)
        {
            return std::abs(wrap_angle(a - b));
        };

        const auto state = get_current_state();
        const float x = state[2];
        const float y = state[3];
        const float theta = wrap_angle(state[4]);

        set_robot_pose(x, y, theta, false);
        const float sdf_theta = evaluate_pose_mean_abs_sdf(lidar_points);

        const float theta_pi = wrap_angle(theta + static_cast<float>(M_PI));
        set_robot_pose(x, y, theta_pi, false);
        const float sdf_theta_pi = evaluate_pose_mean_abs_sdf(lidar_points);

        const bool similar_fit = std::isfinite(sdf_theta) && std::isfinite(sdf_theta_pi)
                                 && std::abs(sdf_theta - sdf_theta_pi) <= kYawTieTolerance;
        const float d_theta = ang_dist(theta, prior_phi);
        const float d_theta_pi = ang_dist(theta_pi, prior_phi);

        if ((sdf_theta_pi < sdf_theta) || (similar_fit && d_theta_pi < d_theta))
            set_robot_pose(x, y, theta_pi, false);
        else
            set_robot_pose(x, y, theta, false);
    }

    bool RoomConcept::bootstrap_initialization_from_lidar()
    {
        if (is_initialized())
            return true;
        if (run_ctx_.sensor_buffer == nullptr)
            return false;

        Eigen::Vector2f init_xy = Eigen::Vector2f::Zero();
        float init_phi = 0.f;
        bool have_saved_pose = false;

        // Try saved pose first — if available we can initialize the model immediately
        // without needing a lidar scan (lidar is only required for pose estimation).
        if (!seed_pose_file_path_.empty())
        {
            std::ifstream in(seed_pose_file_path_);
            float sx = 0.f, sy = 0.f, st = 0.f;
            if (in.is_open() && (in >> sx >> sy >> st))
            {
                init_xy = Eigen::Vector2f(sx, sy);
                init_phi = st;
                have_saved_pose = true;
                qInfo() << "RoomConcept bootstrap: using saved seed pose" << sx << sy << st;
            }
            else
            {
                qWarning() << "RoomConcept bootstrap: seed pose not loaded from"
                           << QString::fromStdString(seed_pose_file_path_)
                           << "(file missing or parse error)";
            }
        }

        // When we have a saved pose the model can be seeded without lidar.
        // When we have no saved pose we need lidar to estimate the initial position.
        std::vector<Eigen::Vector3f> pts;
        if (!have_saved_pose)
        {
            const auto& [gt_, lidar_high_] = run_ctx_.sensor_buffer->read_last();
            if (!lidar_high_.has_value() || lidar_high_->first.empty())
                return false;
            pts = lidar_high_->first;

            PointcloudCenterEstimator estimator;
            Eigen::Vector2d room_center_in_robot = Eigen::Vector2d::Zero();
            if (const auto obb = estimator.estimate_obb(pts); obb.has_value())
            {
                room_center_in_robot = obb->center;
                init_phi = static_cast<float>(obb->rotation);
            }
            else if (const auto c = estimator.estimate(pts); c.has_value())
            {
                room_center_in_robot = c.value();
                init_phi = estimate_orientation_from_points(pts);
            }

            Eigen::Rotation2Df R(init_phi);
            init_xy = -(R * room_center_in_robot.cast<float>());
        }

        if (init_use_polygon_ && init_polygon_vertices_.size() >= 3)
        {
            set_polygon_room(init_polygon_vertices_);
            set_robot_pose(init_xy.x(), init_xy.y(), init_phi, false);
        }
        else
        {
            set_initial_state(init_room_width_, init_room_length_, init_xy.x(), init_xy.y(), init_phi);
        }

        bool used_grid_search = false;
        if (!have_saved_pose)
        {
            qInfo() << "RoomConcept bootstrap: no saved seed pose. Running initial grid search.";
            used_grid_search = grid_search_initial_pose(pts, 0.5f, static_cast<float>(M_PI_4));
            if (!used_grid_search)
                qWarning() << "RoomConcept bootstrap: grid search failed. Keeping estimator-based initialization.";
        }

        if (!used_grid_search)
            resolve_initial_yaw_ambiguity(pts, init_phi);

        return true;
    }

    void RoomConcept::run()
    {
        qInfo() << "[LocThread] Localization thread started";
        loc_running_ = true;

        auto wait_period = std::chrono::milliseconds(40);
        std::int64_t last_ts = -1;
        constexpr auto kMinWait = std::chrono::milliseconds(2);
        constexpr auto kMaxWait = std::chrono::milliseconds(100);

        while (!stop_requested_.load())
        {
            // ===== 1. DRAIN PENDING COMMANDS =====
            {
                std::vector<Command> cmds;
                {
                    std::lock_guard lock(cmd_mutex_);
                    cmds.swap(pending_commands_);
                }
                for (auto& cmd : cmds)
                {
                    std::visit([this](auto&& arg)
                    {
                        using T = std::decay_t<decltype(arg)>;
                        if constexpr (std::is_same_v<T, CmdSetPolygon>)
                            set_polygon_room(arg.vertices);
                        else if constexpr (std::is_same_v<T, CmdSetPose>)
                            set_robot_pose(arg.x, arg.y, arg.theta);
                        else if constexpr (std::is_same_v<T, CmdGridSearch>)
                            grid_search_initial_pose(arg.lidar_points, arg.grid_res, arg.angle_res);
                    }, cmd);
                }
            }

            // ===== 2. CHECK INITIALIZATION =====
            if (!is_initialized())
            {
                bootstrap_initialization_from_lidar();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                std::cout << "[LocThread] Waiting for initialization..." << std::endl;
                continue;
            }
            
            // ===== 3. READ LIDAR DATA =====
            const auto& [gt_, lidar_high_] = run_ctx_.sensor_buffer->read_last();
            if (!lidar_high_.has_value())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            // ===== 4. SKIP IF SAME LIDAR FRAME =====
            const auto current_ts = lidar_high_->second;
            if (current_ts == last_ts)
            {
                std::this_thread::sleep_for(kMinWait);
                continue;
            }

            // ===== 5. SNAPSHOT VELOCITY & ODOMETRY HISTORY =====
            auto vel_snap  = run_ctx_.velocity_buffer->get_snapshot<0>();
            auto odom_snap = run_ctx_.odometry_buffer->get_snapshot<0>();

            // ===== 6. RUN LOCALIZATION UPDATE =====
            const auto t_update_start = std::chrono::high_resolution_clock::now();
            const auto res = update(lidar_high_.value(), vel_snap, odom_snap);
            const float update_ms = std::chrono::duration<float, std::milli>(
                std::chrono::high_resolution_clock::now() - t_update_start).count();

            // ===== 7. PUBLISH RESULT =====
            {
                std::lock_guard lock(result_mutex_);
                last_result_ = res;
            }
            if (res.ok && !loc_initialized_.load())
                loc_initialized_ = true;

            // ===== 8. RECOVERY DETECTION =====
            // Use sdf_mse (sqrt = avg SDF error in meters) — not final_loss which is
            // the full RFE cost including prior/motion terms and can be in the hundreds.
            if (res.iterations_used > 0 && recovery_cooldown_ == 0)
            {
                const float avg_sdf_err = std::sqrt(res.sdf_mse);  // meters
                if (avg_sdf_err > params.recovery_loss_threshold)
                {
                    consecutive_bad_frames_++;
                    if (consecutive_bad_frames_ >= params.recovery_consecutive_count)
                    {
                        qWarning() << "[LocThread] Recovery triggered after" << consecutive_bad_frames_
                                   << "bad frames. avg_sdf_err=" << avg_sdf_err << "m"
                                   << "Running grid search...";
                        const auto& pts = lidar_high_->first;
                        grid_search_initial_pose(pts, 0.5f, static_cast<float>(M_PI_4));
                        window_.clear();
                        boundary_prior_.valid = false;
                        consecutive_bad_frames_ = 0;
                        recovery_cooldown_ = 30;  // skip recovery checks for 30 frames
                        qInfo() << "[LocThread] Recovery complete.";
                    }
                }
                else
                {
                    consecutive_bad_frames_ = 0;  // reset on good frame
                }
            }
            if (recovery_cooldown_ > 0) recovery_cooldown_--;

            last_ts = current_ts;
            wait_period = std::max(wait_period - std::chrono::milliseconds(1), kMinWait);
            update_ms_accum_ += update_ms;
            update_ms_count_++;
            const float avg_update_ms = update_ms_accum_ / update_ms_count_;
            const int fps = loc_fps_.print("[LocThread] update_ms(last/avg)=" + std::to_string(static_cast<int>(update_ms))
                               + "/" + std::to_string(static_cast<int>(avg_update_ms)), 2000);
            if (fps > 0) { update_ms_accum_ = 0.f; update_ms_count_ = 0; }
            std::this_thread::sleep_for(wait_period);
        }

        loc_running_ = false;
        qInfo() << "[LocThread] Localization thread stopped";
    }

    float RoomConcept::find_best_initial_orientation(const std::vector<Eigen::Vector3f>& lidar_points,
                                                        float x, float y, float base_phi)
    {
        if (model_ == nullptr || lidar_points.empty())
            return base_phi;

        // Test 4 orientations: base, +90°, +180°, +270° (covers symmetries)
        const std::vector<float> angle_offsets = {0.0f, M_PI_2, M_PI, 3.0f * M_PI_2};

        // Also test mirrored positions (x, y) and (-x, y) with all rotations
        // This handles axis-mirroring ambiguity
        const std::vector<std::pair<float, float>> position_variants = {
            {x, y},      // Original
            {-x, y},     // Mirror X
            {x, -y},     // Mirror Y
            {-x, -y}     // Mirror both
        };

        // Subsample points for faster evaluation
        std::vector<Eigen::Vector3f> sample_points;
        const int max_samples = 100;
        const int stride = std::max(1, static_cast<int>(lidar_points.size()) / max_samples);
        for (size_t i = 0; i < lidar_points.size(); i += stride)
            sample_points.push_back(lidar_points[i]);

        const torch::Tensor points_tensor = points_to_tensor_xyz(sample_points);

        float best_phi = base_phi;
        float best_x = x;
        float best_y = y;
        float best_loss = std::numeric_limits<float>::infinity();

        // Test all combinations of position variants and angle offsets
        for (const auto& [test_x, test_y] : position_variants)
        {
            for (float offset : angle_offsets)
            {
                float test_phi = base_phi + offset;
                // Normalize to [-pi, pi]
                while (test_phi > M_PI) test_phi -= 2.0f * M_PI;
                while (test_phi < -M_PI) test_phi += 2.0f * M_PI;

                // Temporarily set the pose
                model_->robot_pos.data().copy_(torch::tensor({test_x, test_y},
                    torch::TensorOptions().device(get_device())));
                model_->robot_theta.data().copy_(torch::tensor({test_phi},
                    torch::TensorOptions().device(get_device())));

                // Evaluate SDF loss (without gradient)
                torch::NoGradGuard no_grad;
                const auto sdf_vals = model_->sdf(points_tensor);
                const float loss = torch::mean(torch::square(sdf_vals)).item<float>();

                qDebug() << "  Testing pos=(" << test_x << "," << test_y << ") phi="
                         << qRadiansToDegrees(test_phi) << "° -> loss=" << loss;

                if (loss < best_loss)
                {
                    best_loss = loss;
                    best_phi = test_phi;
                    best_x = test_x;
                    best_y = test_y;
                }
            }
        }

        // Set the best position found
        model_->robot_pos.data().copy_(torch::tensor({best_x, best_y},
            torch::TensorOptions().device(get_device())));

        qInfo() << "Best initial pose: (" << best_x << "," << best_y << ") phi="
                << qRadiansToDegrees(best_phi) << "° (loss=" << best_loss << ")";
        return best_phi;
    }

    bool RoomConcept::grid_search_initial_pose(const std::vector<Eigen::Vector3f>& lidar_points,
                                                  float grid_resolution,
                                                  float angle_resolution)
    {
        if (model_ == nullptr || lidar_points.empty())
            return false;

        qInfo() << "Starting grid search for initial pose...";

        // Get room bounds from polygon or half_extents
        float min_x, max_x, min_y, max_y;
        if (model_->use_polygon && model_->polygon_vertices.defined())
        {
            auto verts_cpu = model_->polygon_vertices.to(torch::kCPU);
            auto acc = verts_cpu.accessor<float, 2>();
            min_x = max_x = acc[0][0];
            min_y = max_y = acc[0][1];
            for (int i = 1; i < verts_cpu.size(0); i++)
            {
                min_x = std::min(min_x, acc[i][0]);
                max_x = std::max(max_x, acc[i][0]);
                min_y = std::min(min_y, acc[i][1]);
                max_y = std::max(max_y, acc[i][1]);
            }
        }
        else
        {
            auto he_cpu = model_->half_extents.to(torch::kCPU);
            float hw = he_cpu[0].item<float>();
            float hh = he_cpu[1].item<float>();
            min_x = -hw; max_x = hw;
            min_y = -hh; max_y = hh;
        }

        // Add margin inside the room (robot shouldn't be at the walls)
        const float margin = 0.3f;  // 30cm from walls
        min_x += margin; max_x -= margin;
        min_y += margin; max_y -= margin;

        // Subsample lidar points for faster evaluation
        std::vector<Eigen::Vector3f> sample_points;
        const int max_samples = 150;
        const int stride = std::max(1, static_cast<int>(lidar_points.size()) / max_samples);
        for (size_t i = 0; i < lidar_points.size(); i += stride)
            sample_points.push_back(lidar_points[i]);

        const torch::Tensor points_tensor = points_to_tensor_xyz(sample_points, get_device());

        // Generate angle candidates
        std::vector<float> angles;
        for (float a = -M_PI; a < M_PI; a += angle_resolution)
            angles.push_back(a);

        float best_x = 0, best_y = 0, best_theta = 0;
        float best_loss = std::numeric_limits<float>::infinity();
        int total_tests = 0;

        // Grid search
        for (float x = min_x; x <= max_x; x += grid_resolution)
        {
            for (float y = min_y; y <= max_y; y += grid_resolution)
            {
                for (float theta : angles)
                {
                    // Set test pose
                    model_->robot_pos.data().copy_(torch::tensor({x, y},
                        torch::TensorOptions().device(get_device())));
                    model_->robot_theta.data().copy_(torch::tensor({theta},
                        torch::TensorOptions().device(get_device())));

                    // Evaluate SDF loss
                    torch::NoGradGuard no_grad;
                    const auto sdf_vals = model_->sdf(points_tensor);
                    const float loss = torch::mean(torch::square(sdf_vals)).item<float>();

                    if (loss < best_loss)
                    {
                        best_loss = loss;
                        best_x = x;
                        best_y = y;
                        best_theta = theta;
                    }
                    total_tests++;
                }
            }
        }

        // Set the best pose found
        model_->robot_pos.data().copy_(torch::tensor({best_x, best_y},
            torch::TensorOptions().device(get_device())));
        model_->robot_theta.data().copy_(torch::tensor({best_theta},
            torch::TensorOptions().device(get_device())));

        // Initialize smoothed pose
        smoothed_pose_ = Eigen::Vector3f(best_x, best_y, best_theta);
        has_smoothed_pose_ = true;

        // Reset tracking state
        needs_orientation_search_ = false;  // We already found the best orientation
        tracking_step_count_ = 0;
        current_covariance = Eigen::Matrix3f::Identity() * 0.1f;

        qInfo() << "Grid search complete:" << total_tests << "poses tested";
        qInfo() << "Best pose: (" << best_x << "," << best_y << ") theta="
                << qRadiansToDegrees(best_theta) << "° (loss=" << best_loss << ")";

        // Return true if we found a reasonable pose (loss < threshold)
        return best_loss < 1.0f;  // Threshold for "good" pose
    }

    void RoomConcept::set_initial_state(float width, float length, float x, float y, float phi)
    {
        model_ = std::make_shared<Model>();
        model_->set_device(get_device());  // Set device before init
        model_->init_from_state(width, length, x, y, phi, params.wall_height);
        needs_orientation_search_ = true;  // Will search for best orientation on first update
        has_smoothed_pose_ = false;  // Reset smoothing
        tracking_step_count_ = 0;  // Reset early exit tracking
        prediction_early_exits_ = 0;
        current_velocity_weights_ = Eigen::Vector3f::Ones();  // Reset velocity weights
        window_.clear(); boundary_prior_.valid = false;  // Clear RFE window

        qInfo() << "RoomConcept using device:" << (get_device() == torch::kCUDA ? "CUDA" : "CPU");
    }


    void RoomConcept::set_polygon_room(const std::vector<Eigen::Vector2f>& polygon_vertices)
    {
        if (polygon_vertices.size() < 3)
        {
            std::cerr << "set_polygon_room: Need at least 3 vertices" << std::endl;
            return;
        }

        // Vertices are in room frame (where user clicked on the viewer)
        // Keep current robot pose if we have one, otherwise start at origin
        float init_x = 0.0f;
        float init_y = 0.0f;
        float init_phi = 0.0f;

        if (model_ != nullptr)
        {
            // Preserve current robot pose
            const auto state = model_->get_state();
            init_x = state[2];
            init_y = state[3];
            init_phi = state[4];
        }

        model_ = std::make_shared<Model>();
        model_->set_device(get_device());  // Set device before init
        model_->init_from_polygon(polygon_vertices, init_x, init_y, init_phi, params.wall_height);

        // Reset state but keep covariance reasonable
        last_lidar_timestamp = 0;
        last_update_result = UpdateResult{};
        current_covariance = Eigen::Matrix3f::Identity() * 0.1f;
        needs_orientation_search_ = true;  // Will search for best orientation on first update
        has_smoothed_pose_ = false;  // Reset smoothing
        tracking_step_count_ = 0;  // Reset early exit tracking
        prediction_early_exits_ = 0;
        current_velocity_weights_ = Eigen::Vector3f::Ones();  // Reset velocity weights
        window_.clear(); boundary_prior_.valid = false;  // Clear RFE window

        qInfo() << "RoomConcept initialized with polygon room:" << polygon_vertices.size()
                << "vertices. Robot at (" << init_x << "," << init_y << "," << init_phi << ")";
    }

    void RoomConcept::set_robot_pose(float x, float y, float theta, bool manual_reset)
    {
        if (model_ == nullptr)
        {
            qWarning() << "Cannot set robot pose: model not initialized";
            return;
        }

        // Directly update robot pose tensors
        model_->robot_pos = torch::tensor({x, y}, torch::TensorOptions().dtype(torch::kFloat32).device(model_->device_).requires_grad(true));
        model_->robot_theta = torch::tensor({theta}, torch::TensorOptions().dtype(torch::kFloat32).device(model_->device_).requires_grad(true));

        // Reset smoothed pose to new position
        smoothed_pose_ = Eigen::Vector3f(x, y, theta);
        has_smoothed_pose_ = true;

        // Reset covariance to reasonable value (we're uncertain after manual placement)
        current_covariance = Eigen::Matrix3f::Identity() * 0.1f;

        // Reset tracking counters and prediction state
        tracking_step_count_ = 0;
        needs_orientation_search_ = false;  // User explicitly set orientation
        last_lidar_timestamp = 0;  // Force fresh start

        // Optional skip optimization for a few frames to let manual reset settle.
        manual_reset_frames_ = manual_reset ? 5 : 0;

        // Clear any previous prediction
        model_->has_prediction = false;
        model_->robot_prev_pose = std::nullopt;

        // Clear sliding window (stale poses are invalid after manual reset)
        window_.clear();
        boundary_prior_.valid = false;

        // Reset last update result with new pose
        last_update_result = UpdateResult{};
        last_update_result.robot_pose.translation() = Eigen::Vector2f(x, y);
        last_update_result.robot_pose.linear() = Eigen::Rotation2Df(theta).toRotationMatrix();
        last_update_result.ok = true;
    }

    float RoomConcept::evaluate_pose_mean_abs_sdf(const std::vector<Eigen::Vector3f>& lidar_points,
                                                  int max_samples) const
    {
        if (model_ == nullptr || lidar_points.empty())
            return std::numeric_limits<float>::infinity();

        std::vector<Eigen::Vector3f> sampled;
        const int cap = std::max(1, max_samples);
        if (static_cast<int>(lidar_points.size()) > cap)
        {
            const int stride = static_cast<int>(lidar_points.size()) / cap;
            sampled.reserve(cap);
            for (size_t i = 0; i < lidar_points.size(); i += stride)
                sampled.push_back(lidar_points[i]);
        }
        else
        {
            sampled = lidar_points;
        }

        torch::NoGradGuard no_grad;
        const auto points_tensor = points_to_tensor_xyz(sampled, get_device());
        const auto sdf_vals = model_->sdf(points_tensor);
        return torch::mean(torch::abs(sdf_vals)).item<float>();
    }

    Eigen::Vector3f RoomConcept::compute_velocity_adaptive_weights(const OdometryPrior& odometry_prior)
    {
        /**
         * Compute velocity-adaptive precision weights for [x, y, theta].
         *
         * Based on the current velocity profile:
         * - If rotating (high angular, low linear): boost theta weight, reduce x,y
         * - If moving straight (high linear, low angular): boost x,y, reduce theta
         * - If stationary: use base weights (uniform)
         *
         * The weights scale gradients during optimization, making the system
         * more responsive to parameters expected to change based on motion.
         */
        if (!params.velocity_adaptive_weights || !odometry_prior.valid)
        {
            return Eigen::Vector3f::Ones();
        }

        // Get velocities from odometry prior (dt is in milliseconds, convert to seconds)
        const float dt_sec = std::max(odometry_prior.dt / 1000.0f, 0.001f);
        const float linear_speed = odometry_prior.delta_pose.head<2>().norm() / dt_sec;
        const float angular_speed = std::abs(odometry_prior.delta_pose[2]) / dt_sec;

        // Determine motion profile
        const bool is_rotating = angular_speed > params.angular_velocity_threshold;
        const bool is_translating = linear_speed > params.linear_velocity_threshold;

        float w_x, w_y, w_theta;

        if (is_rotating && !is_translating)
        {
            // Pure rotation: emphasize theta, de-emphasize x, y
            w_x = params.weight_reduction_factor;
            w_y = params.weight_reduction_factor;
            w_theta = params.weight_boost_factor;
        }
        else if (is_translating && !is_rotating)
        {
            // Pure translation: emphasize x, y based on direction
            // Get velocity direction in robot frame
            const float vx = odometry_prior.delta_pose[0] / std::max(odometry_prior.dt, 0.001f);
            const float vy = odometry_prior.delta_pose[1] / std::max(odometry_prior.dt, 0.001f);

            if (std::abs(vy) > std::abs(vx))
            {
                // Mostly forward/backward motion - emphasize y (forward axis)
                w_x = 1.0f;
                w_y = params.weight_boost_factor;
            }
            else
            {
                // Mostly lateral motion - emphasize x
                w_x = params.weight_boost_factor;
                w_y = 1.0f;
            }
            w_theta = params.weight_reduction_factor;
        }
        else if (is_rotating && is_translating)
        {
            // Combined motion: moderate boost for all
            w_x = 1.2f;
            w_y = 1.2f;
            w_theta = 1.2f;
        }
        else
        {
            // Stationary: use base weights
            w_x = 1.0f;
            w_y = 1.0f;
            w_theta = 1.0f;
        }

        Eigen::Vector3f new_weights(w_x, w_y, w_theta);

        // Smooth transition using exponential moving average
        const float alpha = params.weight_smoothing_alpha;
        current_velocity_weights_ = (1.0f - alpha) * current_velocity_weights_ + alpha * new_weights;

        return current_velocity_weights_;
    }

    RoomConcept::UpdateResult RoomConcept::update(
                const std::pair<std::vector<Eigen::Vector3f>, int64_t> &lidar,
                const std::vector<VelocityCommand> &velocity_history,
                const std::vector<OdometryReading> &odometry_history)
    {
        UpdateResult res;
        if(lidar.first.empty())
            return res;

        if(model_ == nullptr)
            return res;

        // ===== MANUAL RESET: Skip optimization and use current pose =====
        if (manual_reset_frames_ > 0)
        {
            manual_reset_frames_--;

            // Just return current pose without optimization
            const auto state = model_->get_state();
            res.ok = true;
            res.state = state;
            res.robot_pose.translation() = Eigen::Vector2f(state[2], state[3]);
            res.robot_pose.linear() = Eigen::Rotation2Df(state[4]).toRotationMatrix();
            res.covariance = current_covariance;
            res.final_loss = 0.0f;
            res.sdf_mse = 0.0f;
            res.timestamp_ms = lidar.second;

            last_update_result = res;
            last_lidar_timestamp = lidar.second;

            qInfo() << "Manual reset: skipping optimization (" << manual_reset_frames_ << " frames remaining)";
            return res;
        }

        // ===== ORIENTATION SEARCH ON FIRST UPDATE =====
        // Test multiple orientations (0°, 90°, 180°, 270°) to avoid symmetry traps
        if (needs_orientation_search_)
        {
            const auto state = model_->get_state();
            const float best_phi = find_best_initial_orientation(lidar.first, state[2], state[3], state[4]);
            model_->robot_theta.data().copy_(torch::tensor({best_phi},
                torch::TensorOptions().device(get_device())));
            needs_orientation_search_ = false;
            qInfo() << "Initial orientation search complete. Using phi=" << qRadiansToDegrees(best_phi) << "°";
        }

        // ===== ODOMETRY PRIOR BETWEEN LIDAR MEASUREMENTS =====
        auto odometry_prior = compute_odometry_prior(velocity_history, lidar);

        // ===== PREDICT STEP =====
        // Propagates state: x_pred = x_prev + f(u, dt)
        // Propagates covariance: P_pred = F*P*F^T + Q where F is motion model Jacobian
        const PredictionState prediction = predict_step(model_, odometry_prior, true);

        // ===== UPDATE PRIOR COVARIANCE WITH PROPAGATED VALUE =====
        // EKF predict: P_prior = F * P_prev * F^T + Q
        if (prediction.have_propagated && prediction.propagated_cov.defined())
        {
            // Convert propagated covariance from torch tensor to Eigen (must be on CPU for accessor)
            auto cov_cpu = prediction.propagated_cov.to(torch::kCPU);
            auto cov_acc = cov_cpu.accessor<float, 2>();
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    current_covariance(i, j) = cov_acc[i][j];
        }

        // ===== ACTIVE INFERENCE: Dual-prior fusion (command + odometry) =====
        Eigen::Vector2f pred_pos;
        float pred_theta;

        // Default: use model's current state (first call or no valid odometry)
        {
            auto state = model_->get_state();
            pred_pos = Eigen::Vector2f(state[2], state[3]);
            pred_theta = state[4];
        }

        if (last_update_result.ok && odometry_prior.valid)
        {
            // --- Command prior: predict from commanded velocity ---
            const Eigen::Vector2f cmd_pos = last_update_result.robot_pose.translation()
                     + odometry_prior.delta_pose.head<2>();
            float cmd_theta = std::atan2(last_update_result.robot_pose.linear()(1, 0),
                                    last_update_result.robot_pose.linear()(0, 0))
                       + odometry_prior.delta_pose[2];
            while (cmd_theta > M_PI) cmd_theta -= 2.0f * M_PI;
            while (cmd_theta < -M_PI) cmd_theta += 2.0f * M_PI;

            const Eigen::Vector3f pred_cmd(cmd_pos.x(), cmd_pos.y(), cmd_theta);

            // --- Measured odometry prior: predict from encoder/IMU readings ---
            auto measured_prior = compute_measured_odometry_prior(odometry_history, lidar);

            if (measured_prior.valid)
            {
                const Eigen::Vector2f odom_pos = last_update_result.robot_pose.translation()
                         + measured_prior.delta_pose.head<2>();
                float odom_theta = std::atan2(last_update_result.robot_pose.linear()(1, 0),
                                        last_update_result.robot_pose.linear()(0, 0))
                           + measured_prior.delta_pose[2];
                while (odom_theta > M_PI) odom_theta -= 2.0f * M_PI;
                while (odom_theta < -M_PI) odom_theta += 2.0f * M_PI;

                const Eigen::Vector3f pred_odom(odom_pos.x(), odom_pos.y(), odom_theta);

                // Compute covariances: command prior is less trusted, odometry is tighter
                const Eigen::Matrix3f cov_cmd = compute_motion_covariance(odometry_prior, false);
                const Eigen::Matrix3f cov_odom = compute_motion_covariance(measured_prior, true);

                // Fuse into single Gaussian: μ_fused, Σ_fused
                const auto [fused_mean, fused_precision] = fuse_priors(
                    pred_cmd, cov_cmd, pred_odom, cov_odom);

                pred_pos = fused_mean.head<2>();
                pred_theta = fused_mean[2];

                // Initialize optimizer state with fused prediction
                model_->robot_pos.data().copy_(torch::tensor({pred_pos.x(), pred_pos.y()},
                    torch::TensorOptions().device(get_device())));
                model_->robot_theta.data().copy_(torch::tensor({pred_theta},
                    torch::TensorOptions().device(get_device())));

                // Set fused prediction for prior loss computation
                model_->set_prediction(pred_pos, pred_theta, fused_precision);
            }
            else
            {
                // No odometry available: fall back to command-only prior
                pred_pos = cmd_pos;
                pred_theta = cmd_theta;

                model_->robot_pos.data().copy_(torch::tensor({pred_pos.x(), pred_pos.y()},
                    torch::TensorOptions().device(get_device())));
                model_->robot_theta.data().copy_(torch::tensor({pred_theta},
                    torch::TensorOptions().device(get_device())));

                Eigen::Matrix3f prior_precision = current_covariance.inverse();
                model_->set_prediction(pred_pos, pred_theta, prior_precision);
            }
        }

        // ===== MINIMISATION STEP (Minimize Realized Free Energy over sliding window) =====
        // Paper: "Total-Time Active Inference", Section 6.4, Algorithm 1
        //
        // Instead of optimising a single pose S_t against a single scan O_t,
        // we jointly optimise the last W poses S_{t-W:t} against their respective
        // scans O_{t-W:t}, linked by motion factors and a boundary prior that
        // summarises discarded history.

        // Subsample lidar points for the newest slot
        const auto& all_points = lidar.first;
        std::vector<Eigen::Vector3f> sampled_points;
        if (static_cast<int>(all_points.size()) > params.max_lidar_points)
        {
            const int stride = static_cast<int>(all_points.size()) / params.max_lidar_points;
            sampled_points.reserve(params.max_lidar_points);
            for (size_t i = 0; i < all_points.size(); i += stride)
                sampled_points.push_back(all_points[i]);
        }
        else
        {
            sampled_points = all_points;
        }

        const torch::Tensor points_tensor = points_to_tensor_xyz(sampled_points, get_device());

        // Increment tracking step counter
        tracking_step_count_++;

        // ===== COMPUTE ODOMETRY DELTA & COVARIANCE FOR NEW SLOT =====
        Eigen::Vector3f slot_odom_delta = Eigen::Vector3f::Zero();
        Eigen::Matrix3f slot_motion_cov = Eigen::Matrix3f::Identity() * 0.01f;
        if (odometry_prior.valid)
        {
            slot_odom_delta = odometry_prior.delta_pose;
            slot_motion_cov = compute_motion_covariance(odometry_prior, false);

            // If measured odometry is available, try to get tighter covariance
            // (This is done during dual-prior fusion above which already set pred_pos/pred_theta)
        }

        // ===== CREATE NEW WINDOW SLOT =====
        WindowSlot new_slot;
        new_slot.pose = torch::tensor({pred_pos.x(), pred_pos.y(), pred_theta},
            torch::TensorOptions().dtype(torch::kFloat32).device(get_device()).requires_grad(true));
        new_slot.lidar_points = points_tensor;
        new_slot.odometry_delta = slot_odom_delta;
        new_slot.motion_cov = slot_motion_cov;
        new_slot.timestamp_ms = lidar.second;

        // ===== SLIDE THE WINDOW (if full) =====
        // Only recompute the boundary prior when Adam will actually run
        // (early exit path returns before reaching Adam, so boundary prior recomputation there is wasted)
        const bool window_will_slide = (static_cast<int>(window_.size()) >= params.rfe_window_size);
        if (window_will_slide)
        {
            // Cheap slide: just pop the oldest slot and copy its pose as boundary mu.
            // Full Hessian recomputation happens only after Adam runs (see end of update()).
            auto pose_cpu = window_.front().pose.detach().to(torch::kCPU);
            auto pose_acc = pose_cpu.accessor<float, 1>();
            boundary_prior_.mu = Eigen::Vector3f(pose_acc[0], pose_acc[1], pose_acc[2]);
            window_.pop_front();
        }

        // ===== APPEND NEW SLOT =====
        window_.push_back(std::move(new_slot));

        // ===== SUBSAMPLE OLDER SLOTS' LIDAR for efficiency =====
        // Keep full resolution only for the newest slot
        if (window_.size() > 1 && params.rfe_max_lidar_per_old_slot > 0)
        {
            for (size_t i = 0; i < window_.size() - 1; i++)
            {
                auto& slot = window_[i];
                const int64_t n_pts = slot.lidar_points.size(0);
                if (n_pts > params.rfe_max_lidar_per_old_slot)
                {
                    // Subsample by stride
                    const int64_t stride = n_pts / params.rfe_max_lidar_per_old_slot;
                    auto indices = torch::arange(0, n_pts, stride,
                        torch::TensorOptions().dtype(torch::kLong).device(slot.lidar_points.device()));
                    slot.lidar_points = slot.lidar_points.index_select(0, indices).contiguous();
                }
            }
        }

        // ===== PREDICTION-BASED EARLY EXIT =====
        // If the predicted pose already has low SDF error, skip optimization entirely.
        // The SDF value is a direct measurement of pose quality — use it alone as the gate.
        // Do NOT gate on covariance: covariance only models uncertainty, but if SDF is low
        // the pose is actually good regardless of what the covariance says. Gating on covariance
        // causes a sawtooth cycle where process noise accumulates until the gate blocks early exit,
        // Adam runs and resets covariance, then the cycle repeats.
        // Inhibit early exit when angular motion is significant: the predicted
        // pose is unreliable during fast rotation and skipping Adam causes the
        // SDF to spike and the drawn lidar to lag behind the room model.
        const float angular_delta = std::abs(slot_odom_delta[2]);
        const bool fast_rotation = angular_delta > params.angular_velocity_threshold * std::max(odometry_prior.dt / 1000.0f, 0.001f);

        if (params.prediction_early_exit &&
            last_update_result.ok &&
            odometry_prior.valid &&
            tracking_step_count_ > params.min_tracking_steps &&
            !fast_rotation)
        {
            {
                torch::NoGradGuard no_grad;
                const auto& newest = window_.back();
                auto pose_xy = newest.pose.index({torch::indexing::Slice(0, 2)});
                auto pose_th = newest.pose.index({torch::indexing::Slice(2, 3)});
                const auto sdf_pred = model_->sdf_at_pose(points_tensor, pose_xy, pose_th);
                const float mean_sdf_pred = torch::mean(torch::abs(sdf_pred)).item<float>();

                const float prediction_trust_threshold = params.sigma_sdf * params.prediction_trust_factor;

                if (mean_sdf_pred < prediction_trust_threshold)
                {
                    prediction_early_exits_++;

                    // Extract pose from newest slot
                    auto pose_cpu = newest.pose.detach().to(torch::kCPU);
                    auto p_acc = pose_cpu.accessor<float, 1>();
                    const float x = p_acc[0], y = p_acc[1], phi = p_acc[2];

                    res.ok = true;
                    res.final_loss = mean_sdf_pred;
                    res.sdf_mse = mean_sdf_pred;
                    res.iterations_used = 0;
                    res.state = model_->get_state();
                    res.state[2] = x; res.state[3] = y; res.state[4] = phi;

                    Eigen::Affine2f pose_aff = Eigen::Affine2f::Identity();
                    pose_aff.translation() = Eigen::Vector2f{x, y};
                    pose_aff.linear() = Eigen::Rotation2Df(phi).toRotationMatrix();
                    res.robot_pose = pose_aff;
                    res.covariance = current_covariance;

                    if (model_->has_prediction)
                    {
                        res.innovation[0] = x - model_->predicted_pos[0].item<float>();
                        res.innovation[1] = y - model_->predicted_pos[1].item<float>();
                        float p_theta = model_->predicted_theta[0].item<float>();
                        res.innovation[2] = phi - p_theta;
                        while (res.innovation[2] > M_PI) res.innovation[2] -= 2.0f * M_PI;
                        while (res.innovation[2] < -M_PI) res.innovation[2] += 2.0f * M_PI;
                        res.innovation_norm = std::sqrt(res.innovation[0]*res.innovation[0] +
                                                        res.innovation[1]*res.innovation[1]);
                    }

                    // Sync model with window pose
                    model_->robot_pos.data().copy_(torch::tensor({x, y},
                        torch::TensorOptions().device(get_device())));
                    model_->robot_theta.data().copy_(torch::tensor({phi},
                        torch::TensorOptions().device(get_device())));
                    model_->robot_prev_pose = res.robot_pose;
                    model_->has_prediction = false;

                    res.timestamp_ms = lidar.second;
                    last_update_result = res;
                    last_lidar_timestamp = lidar.second;

                    if (prediction_early_exits_ % 50 == 0)
                    {
                        qDebug() << "[EARLY_EXIT] Skipped RFE optimization. SDF:" << mean_sdf_pred
                                 << "m, window:" << window_.size()
                                 << ", total early exits:" << prediction_early_exits_;
                    }
                    return res;
                }
            }
        }

        // ===== JOINT OPTIMISATION OVER SLIDING WINDOW (RFE minimisation) =====
        // Collect all window poses as parameters for the optimizer
        auto window_params = collect_window_params();

        // Build optimizer with all window poses
        // Use a single learning rate for the combined [x,y,theta] pose tensor
        const float lr = params.learning_rate_pos;  // dominant LR
        torch::optim::Adam optimizer(
            {torch::optim::OptimizerParamGroup(window_params,
                std::make_unique<torch::optim::AdamOptions>(lr))});

        // Compute velocity-adaptive weights for gradient scaling (applied to newest pose only)
        const Eigen::Vector3f velocity_weights = compute_velocity_adaptive_weights(odometry_prior);

        const int max_iters = params.num_iterations;
        float last_loss = std::numeric_limits<float>::infinity();
        float prev_loss = std::numeric_limits<float>::infinity();
        int iterations = 0;

        for (int i = 0; i < max_iters; ++i)
        {
            optimizer.zero_grad();

            // Compute the full RFE loss over the window
            const torch::Tensor loss = compute_rfe_loss();
            loss.backward();

            // Apply velocity-adaptive gradient weighting to the NEWEST pose only
            {
                torch::NoGradGuard no_grad;
                auto& newest_pose = window_.back().pose;
                if (newest_pose.grad().defined())
                {
                    newest_pose.mutable_grad().index({0}) *= velocity_weights[0];  // x
                    newest_pose.mutable_grad().index({1}) *= velocity_weights[1];  // y
                    newest_pose.mutable_grad().index({2}) *= velocity_weights[2];  // theta
                }
            }

            optimizer.step();

            prev_loss = last_loss;
            last_loss = loss.item<float>();
            iterations = i + 1;

            if (last_loss < params.min_loss_threshold)
                break;
            if (i > 5 && std::abs(prev_loss - last_loss) < 0.01f * prev_loss)
                break;
        }

        // ===== COVARIANCE UPDATE =====
        // Exact Hessian of SDF likelihood via double-backprop (Paper Eq. 28)
        try {
            auto& newest = window_.back();
            auto pose_for_hess = newest.pose.clone().detach().requires_grad_(true);
            auto pose_xy = pose_for_hess.index({torch::indexing::Slice(0, 2)});
            auto pose_theta_h = pose_for_hess.index({torch::indexing::Slice(2, 3)});

            const auto sdf_vals = model_->sdf_at_pose(points_tensor, pose_xy, pose_theta_h);
            const float huber_delta = params.rfe_huber_delta;
            const float inv_var = 1.0f / (params.rfe_obs_sigma * params.rfe_obs_sigma);
            const torch::Tensor likelihood_loss = 0.5f * inv_var *
                torch::nn::functional::huber_loss(
                    sdf_vals,
                    torch::zeros_like(sdf_vals),
                    torch::nn::functional::HuberLossFuncOptions().reduction(torch::kMean).delta(huber_delta)
                );

            Eigen::Matrix3f H_likelihood = autograd_hessian_3x3(likelihood_loss, pose_for_hess);

            // Ensure positive-definiteness: clamp eigenvalues
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(H_likelihood);
            Eigen::Vector3f evals = eig.eigenvalues().cwiseMax(1e-4f);
            H_likelihood = eig.eigenvectors() * evals.asDiagonal() * eig.eigenvectors().transpose();

            Eigen::Matrix3f prior_precision = current_covariance.inverse();
            constexpr float lambda = 1e-4f;
            Eigen::Matrix3f posterior_precision = prior_precision + H_likelihood
                                                 + lambda * Eigen::Matrix3f::Identity();

            Eigen::Matrix3f new_cov = posterior_precision.inverse();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(posterior_precision);
            const auto eigenvalues = solver.eigenvalues();
            const float max_ev = eigenvalues.maxCoeff();
            const float min_ev = eigenvalues.minCoeff();
            res.condition_number = (min_ev > 1e-8f) ? (max_ev / min_ev) : 1e8f;

            if (new_cov.allFinite() && new_cov.determinant() > 1e-10f && res.condition_number < 1e6f)
            {
                current_covariance = new_cov;
                res.covariance = new_cov;
            }
            else
            {
                res.covariance = current_covariance;
            }
        } catch (const std::exception &e) {
            std::cerr << "RFE covariance update failed: " << e.what() << std::endl;
            res.covariance = current_covariance;
            res.condition_number = -1.0f;
        }

        // ===== EXTRACT RESULT FROM NEWEST WINDOW SLOT =====
        res.ok = true;
        res.final_loss = last_loss;
        res.iterations_used = iterations;

        {
            // Read optimised pose from the newest window slot
            auto newest_cpu = window_.back().pose.detach().to(torch::kCPU);
            auto p_acc = newest_cpu.accessor<float, 1>();
            float x = p_acc[0];
            float y = p_acc[1];
            float phi = p_acc[2];

            // Normalize angle
            while (phi > M_PI) phi -= 2.0f * M_PI;
            while (phi < -M_PI) phi += 2.0f * M_PI;

            // Update model's internal pose to match the optimised window result
            // BEFORE computing sdf_mse so it reflects the optimised pose, not the prediction.
            model_->robot_pos.data().copy_(torch::tensor({x, y},
                torch::TensorOptions().device(get_device())));
            model_->robot_theta.data().copy_(torch::tensor({phi},
                torch::TensorOptions().device(get_device())));

            // Now compute sdf_mse at the optimised pose (not prediction)
            res.sdf_mse = compute_sdf_mse_unscaled(points_tensor, *model_);

            res.state = model_->get_state();
            res.state[2] = x;
            res.state[3] = y;
            res.state[4] = phi;

            Eigen::Affine2f pose = Eigen::Affine2f::Identity();
            pose.translation() = Eigen::Vector2f{x, y};
            pose.linear() = Eigen::Rotation2Df(phi).toRotationMatrix();
            res.robot_pose = pose;

            // ===== COMPUTE INNOVATION (Kalman residual) =====
            if (model_->has_prediction)
            {
                res.innovation[0] = x - model_->predicted_pos[0].item<float>();
                res.innovation[1] = y - model_->predicted_pos[1].item<float>();
                float pred_th = model_->predicted_theta[0].item<float>();
                res.innovation[2] = phi - pred_th;
                while (res.innovation[2] > M_PI) res.innovation[2] -= 2.0f * M_PI;
                while (res.innovation[2] < -M_PI) res.innovation[2] += 2.0f * M_PI;
                res.innovation_norm = std::sqrt(res.innovation[0]*res.innovation[0] +
                                                res.innovation[1]*res.innovation[1]);
            }
        }

        // ===== UPDATE MODEL STATE FOR NEXT ITERATION =====
        // Recompute boundary prior with full Hessian now that Adam has refined the poses.
        // This only runs when Adam actually executed (not on early exit).
        if (window_.size() > 0)
            slide_window_boundary_prior_only();

        model_->robot_prev_pose = res.robot_pose;

        auto cov_cpu = torch::zeros({3, 3}, torch::kFloat32);
        auto cov_acc = cov_cpu.accessor<float, 2>();
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                cov_acc[i][j] = res.covariance(i, j);
        model_->prev_cov = cov_cpu.to(get_device());

        model_->has_prediction = false;

        res.timestamp_ms = lidar.second;
        last_update_result = res;
        return res;
    }

    RoomConcept::OdometryPrior RoomConcept::compute_odometry_prior(
             const std::vector<VelocityCommand>& velocity_history,
             const std::pair<std::vector<Eigen::Vector3f>, std::int64_t> &lidar)
    {
         OdometryPrior prior;
         prior.valid = false;
         const auto &[points, lidar_timestamp] = lidar;

         if (last_lidar_timestamp == 0)
         {
             last_lidar_timestamp = lidar_timestamp;
             return prior;
         }

         // Calculate dt
         const auto dt = lidar_timestamp - last_lidar_timestamp;
         if (dt <= 0)
        {
             last_lidar_timestamp = lidar_timestamp;
             return prior;
         }
         prior.dt = dt;

        if (!velocity_history.empty() && last_update_result.ok)
            prior.delta_pose = integrate_velocity_over_window(last_update_result.robot_pose,
                                                              velocity_history,
                                                        last_lidar_timestamp,
                                                        lidar_timestamp);
        else
            // If no history or no valid previous pose, assume STATIONARY (Zero motion)
            // This protects us when sitting still!
            prior.delta_pose = Eigen::Vector3f::Zero();

        prior.valid = true; // ALWAYS valid now


        // Compute covariance
        Eigen::Matrix3f cov_eigen = compute_motion_covariance(prior);
        prior.covariance = torch::eye(3, torch::TensorOptions().dtype(torch::kFloat32).device(get_device()));
        prior.covariance[0][0] = cov_eigen(0, 0);
        prior.covariance[1][1] = cov_eigen(1, 1);
        prior.covariance[2][2] = cov_eigen(2, 2);

        last_lidar_timestamp = lidar_timestamp;
        return prior;
    }

    // ===== HELPER METHOD: Compute motion-based covariance =====
    /**
     * Compute motion-based covariance consistently
     * σ = base + k * distance
     */
    Eigen::Matrix3f RoomConcept::compute_motion_covariance(const OdometryPrior &odometry_prior,
                                                             bool is_measured_odometry)
    {
        // Select noise model: odometry (encoder/IMU) is tighter than commanded velocity
        const float noise_trans = is_measured_odometry ? params.odom_noise_trans : params.cmd_noise_trans;
        const float noise_rot   = is_measured_odometry ? params.odom_noise_rot   : params.cmd_noise_rot;
        const float noise_base  = is_measured_odometry ? params.odom_noise_base  : params.cmd_noise_base;

        float motion_magnitude = std::sqrt(
            odometry_prior.delta_pose[0] * odometry_prior.delta_pose[0] +
            odometry_prior.delta_pose[1] * odometry_prior.delta_pose[1]
        );

        // Uncertainty grows with distance; when stationary use tight constraint
        float base_uncertainty;
        if (motion_magnitude < 0.01f) {
            base_uncertainty = 0.001f;  // 1mm when stationary
        } else {
            base_uncertainty = noise_base;
        }

        float position_std = base_uncertainty + noise_trans * motion_magnitude;
        float rotation_std = 0.01f + noise_rot * std::abs(odometry_prior.delta_pose[2]);

        Eigen::Matrix3f cov = Eigen::Matrix3f::Identity();
        cov(0, 0) = position_std * position_std;
        cov(1, 1) = position_std * position_std;
        cov(2, 2) = rotation_std * rotation_std;

        return cov;
    }

     Eigen::Vector3f RoomConcept::integrate_velocity_over_window(
                const Eigen::Affine2f& robot_pose,
                const std::vector<VelocityCommand> &velocity_history,
                const std::int64_t &t_start_ms,
                const std::int64_t &t_end_ms)
    {
        using clock = std::chrono::high_resolution_clock;
        const auto t_start = clock::time_point(std::chrono::milliseconds(t_start_ms));
        const auto t_end = clock::time_point(std::chrono::milliseconds(t_end_ms));

        Eigen::Vector3f total_delta = Eigen::Vector3f::Zero();

        float running_theta = std::atan2(robot_pose.linear()(1,0), robot_pose.linear()(0,0));

        // Integrate over all velocity commands in [t_start, t_end]
        for (size_t i = 0; i < velocity_history.size(); ++i) {
            const auto&[adv_x, adv_z, rot, timestamp] = velocity_history[i];

            // Get time window for this command
            auto cmd_start = timestamp;
            auto cmd_end = (i + 1 < velocity_history.size())
                           ? velocity_history[i + 1].timestamp
                           : t_end;

            // Clip to [t_start, t_end]
            if (cmd_end < t_start) continue;
            if (cmd_start > t_end) break;

            auto effective_start = std::max(cmd_start, t_start);
            auto effective_end = std::min(cmd_end, t_end);

            const float dt = std::chrono::duration<float>(effective_end - effective_start).count();
            if (dt <= 0) continue;

            // Integrate this segment
            const float dx_local = (adv_x * dt);
            const float dy_local = (adv_z * dt);
            const float dtheta = -rot * dt;  // Negative for right-hand rule

            // Transform to global frame using RUNNING theta
            total_delta[0] += dx_local * std::cos(running_theta) - dy_local * std::sin(running_theta);
            total_delta[1] += dx_local * std::sin(running_theta) + dy_local * std::cos(running_theta);
            total_delta[2] += dtheta;

            // Update running theta for next segment
            running_theta += dtheta;
        }

        return total_delta;
    }

    RoomConcept::PredictionState RoomConcept::predict_step(
                                std::shared_ptr<Model> &room,
                                const OdometryPrior &odometry_prior,
                                bool is_localized)
    {
        int dim = is_localized ? 3 : 5;  // 3 for localized [x,y,theta], 5 for full state [w,h,x,y,theta]
        PredictionState prediction;
        const auto device = get_device();

        // Ensure prev_cov is properly initialized
        if (!room->prev_cov.defined() || room->prev_cov.numel() == 0)
        {
            room->prev_cov = 0.1f * torch::eye(dim, torch::TensorOptions().dtype(torch::kFloat32).device(device));
        }

        // Get current pose for Jacobian computation
        if (not room->robot_prev_pose.has_value())
        {
            // Fallback: simple additive noise
            prediction.propagated_cov = room->prev_cov + params.cmd_noise_trans * params.cmd_noise_trans *
                torch::eye(dim, torch::TensorOptions().dtype(torch::kFloat32).device(device));
            return prediction;
        }

        auto robot_prev_pose = room->robot_prev_pose.value();
        const float theta = std::atan2(robot_prev_pose.linear()(1, 0), robot_prev_pose.linear()(0, 0));

        // Transform global delta back to robot frame for noise computation
        float cos_t = std::cos(theta);
        float sin_t = std::sin(theta);

        float dx_global = odometry_prior.delta_pose[0];
        float dy_global = odometry_prior.delta_pose[1];
        float dtheta = odometry_prior.delta_pose[2];

        // Inverse rotation: robot_frame = R^T * global_frame
        float dx_local = dx_global * cos_t + dy_global * sin_t;
        float dy_local = -dx_global * sin_t + dy_global * cos_t;

        // ===== MOTION MODEL JACOBIAN =====
        // State: [x, y, theta] (for localized) or [w, h, x, y, theta] (for mapping)

        torch::Tensor F = torch::eye(dim, torch::TensorOptions().dtype(torch::kFloat32).device(device));

        if (is_localized) {
            // Jacobian for robot pose only [x, y, theta]
            // ∂x'/∂θ = -dy_local*sin(θ) - dx_local*cos(θ)
            // ∂y'/∂θ =  dy_local*cos(θ) - dx_local*sin(θ)

            F[0][2] = -dy_local * sin_t - dx_local * cos_t;
            F[1][2] =  dy_local * cos_t - dx_local * sin_t;
        } else {
            // Jacobian for full state [w, h, x, y, theta]
            F[2][4] = -dy_local * sin_t - dx_local * cos_t;
            F[3][4] =  dy_local * cos_t - dx_local * sin_t;
        }

        // ===== PROCESS NOISE =====
        // ANISOTROPIC: For Y=forward, X=right coordinate system:
        //   dy_local = FORWARD motion (should get larger noise)
        //   dx_local = LATERAL motion (should get smaller noise)

        float forward_motion = std::abs(dy_local);   // Forward (Y in robot frame)
        float lateral_motion = std::abs(dx_local);   // Lateral (X in robot frame)
        const float angular_motion = std::abs(dtheta);

        // odometry_prior.dt is stored in milliseconds in this pipeline.
        // Convert to seconds so process-noise floor is time-consistent.
        const float dt_s = std::max(0.001f, odometry_prior.dt * 0.001f);
        const float linear_speed = (forward_motion + lateral_motion) / dt_s;
        const float angular_speed = angular_motion / dt_s;
        const bool near_stationary = linear_speed < 0.03f && angular_speed < 0.03f;

        // Base noise grows with elapsed time (random-walk style) and is heavily reduced at rest.
        // This keeps slow covariance inflation when stopped, avoiding constant re-minimization.
        const float time_scale = std::sqrt(std::min(1.0f, 4.0f * dt_s));
        float base_trans_noise = params.cmd_noise_base * time_scale;
        if (near_stationary)
            base_trans_noise *= params.stationary_noise_damping;

        // Forward uncertainty: grows with forward motion
        float forward_noise = base_trans_noise + params.cmd_noise_trans * forward_motion;

        // Lateral uncertainty: smaller for differential drive (30% of forward)
        float lateral_noise = base_trans_noise + 0.3f * params.cmd_noise_trans * lateral_motion;

        // Rotation noise: base + motion-dependent
        float base_rot_noise = 0.5f * base_trans_noise;
        float rot_noise = base_rot_noise + params.cmd_noise_rot * std::abs(dtheta);

        torch::Tensor Q = torch::zeros({dim, dim}, torch::TensorOptions().dtype(torch::kFloat32).device(device));

        if (is_localized) {
            // Build noise covariance in robot frame: Q = diag(lateral², forward², theta²)
            float sigma_x = lateral_noise;   // X = right/lateral (smaller)
            float sigma_y = forward_noise;   // Y = forward (larger)
            float sigma_theta = rot_noise;

            // Transform to global frame: Q_global = R * Q_local * R^T
            // For X=right, Y=forward:
            Q[0][0] = sigma_x*sigma_x * sin_t*sin_t + sigma_y*sigma_y * cos_t*cos_t;
            Q[0][1] = (sigma_y*sigma_y - sigma_x*sigma_x) * cos_t * sin_t;
            Q[1][0] = Q[0][1];
            Q[1][1] = sigma_x*sigma_x * cos_t*cos_t + sigma_y*sigma_y * sin_t*sin_t;
            Q[2][2] = sigma_theta * sigma_theta;
        } else {
            // Full state: room doesn't accumulate noise, only robot pose
            float sigma_x = lateral_noise;
            float sigma_y = forward_noise;
            float sigma_theta = rot_noise;

            Q[2][2] = sigma_y*sigma_y * cos_t*cos_t + sigma_x*sigma_x * sin_t*sin_t;
            Q[2][3] = (sigma_y*sigma_y - sigma_x*sigma_x) * cos_t * sin_t;
            Q[3][2] = Q[2][3];
            Q[3][3] = sigma_y*sigma_y * sin_t*sin_t + sigma_x*sigma_x * cos_t*cos_t;
            Q[4][4] = sigma_theta * sigma_theta;
        }

        // ===== EKF PREDICTION =====
        // P_pred = F * P_prev * F^T + Q
        torch::Tensor propagated = torch::matmul(torch::matmul(F, room->prev_cov), F.t()) + Q;

        prediction.propagated_cov = propagated;
        prediction.have_propagated = true;
        return prediction;
    }

    Eigen::Vector3f RoomConcept::integrate_odometry_over_window(
                const Eigen::Affine2f& robot_pose,
                const std::vector<OdometryReading> &odometry_history,
                const int64_t &t_start_ms,
                const int64_t &t_end_ms)
    {
        using clock = std::chrono::high_resolution_clock;
        const auto t_start = clock::time_point(std::chrono::milliseconds(t_start_ms));
        const auto t_end = clock::time_point(std::chrono::milliseconds(t_end_ms));

        Eigen::Vector3f total_delta = Eigen::Vector3f::Zero();
        float running_theta = std::atan2(robot_pose.linear()(1,0), robot_pose.linear()(0,0));

        // Integrate over all odometry readings in [t_start, t_end]
        for (size_t i = 0; i < odometry_history.size(); ++i)
        {
            const auto& odom = odometry_history[i];

            // Get time window for this reading
            auto cmd_start = odom.timestamp;
            auto cmd_end = (i + 1 < odometry_history.size())
                           ? odometry_history[i + 1].timestamp
                           : t_end;

            // Clip to [t_start, t_end]
            if (cmd_end < t_start) continue;
            if (cmd_start > t_end) break;

            auto effective_start = std::max(cmd_start, t_start);
            auto effective_end = std::min(cmd_end, t_end);

            const float dt = std::chrono::duration<float>(effective_end - effective_start).count();
            if (dt <= 0) continue;

            // Odometry velocities are in robot frame: adv=forward(Y), side=lateral(X), rot=angular
            const float dx_local = odom.side * dt;   // lateral (X in robot frame)
            const float dy_local = odom.adv * dt;    // forward (Y in robot frame)
            const float dtheta = -odom.rot * dt;     // same sign convention as commanded

            // Transform to global frame using running theta
            total_delta[0] += dx_local * std::cos(running_theta) - dy_local * std::sin(running_theta);
            total_delta[1] += dx_local * std::sin(running_theta) + dy_local * std::cos(running_theta);
            total_delta[2] += dtheta;

            running_theta += dtheta;
        }

        return total_delta;
    }

    RoomConcept::OdometryPrior RoomConcept::compute_measured_odometry_prior(
             const std::vector<OdometryReading>& odometry_history,
             const std::pair<std::vector<Eigen::Vector3f>, std::int64_t> &lidar)
    {
        OdometryPrior prior;
        prior.valid = false;
        const auto& [points, lidar_timestamp] = lidar;

        if (last_lidar_timestamp == 0 || odometry_history.empty() || !last_update_result.ok)
            return prior;

        const auto dt = lidar_timestamp - last_lidar_timestamp;
        if (dt <= 0)
            return prior;
        prior.dt = static_cast<float>(dt);

        prior.delta_pose = integrate_odometry_over_window(
            last_update_result.robot_pose,
            odometry_history,
            last_lidar_timestamp,
            lidar_timestamp);

        prior.valid = true;

        // Compute covariance using odometry noise model (tighter than command)
        Eigen::Matrix3f cov_eigen = compute_motion_covariance(prior, true);
        prior.covariance = torch::eye(3, torch::TensorOptions().dtype(torch::kFloat32).device(get_device()));
        prior.covariance[0][0] = cov_eigen(0, 0);
        prior.covariance[1][1] = cov_eigen(1, 1);
        prior.covariance[2][2] = cov_eigen(2, 2);

        return prior;
    }

    std::pair<Eigen::Vector3f, Eigen::Matrix3f> RoomConcept::fuse_priors(
        const Eigen::Vector3f &pred_cmd, const Eigen::Matrix3f &cov_cmd,
        const Eigen::Vector3f &pred_odom, const Eigen::Matrix3f &cov_odom) const
    {
        // Bayesian fusion of two Gaussian priors:
        //   Σ_fused⁻¹ = Σ_cmd⁻¹ + Σ_odom⁻¹
        //   μ_fused = Σ_fused * (Σ_cmd⁻¹ * μ_cmd + Σ_odom⁻¹ * μ_odom)
        constexpr float reg = 1e-6f;
        const Eigen::Matrix3f prec_cmd = (cov_cmd + reg * Eigen::Matrix3f::Identity()).inverse();
        const Eigen::Matrix3f prec_odom = (cov_odom + reg * Eigen::Matrix3f::Identity()).inverse();

        const Eigen::Matrix3f fused_precision = prec_cmd + prec_odom;
        const Eigen::Matrix3f fused_cov = fused_precision.inverse();

        // Handle angle wrapping: compute angular difference carefully
        Eigen::Vector3f pred_odom_adj = pred_odom;
        float angle_diff = pred_odom[2] - pred_cmd[2];
        while (angle_diff > M_PI) angle_diff -= 2.0f * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0f * M_PI;
        pred_odom_adj[2] = pred_cmd[2] + angle_diff;  // Unwrap relative to cmd

        const Eigen::Vector3f fused_mean = fused_cov * (prec_cmd * pred_cmd + prec_odom * pred_odom_adj);

        // Normalize fused angle
        float theta = fused_mean[2];
        while (theta > M_PI) theta -= 2.0f * M_PI;
        while (theta < -M_PI) theta += 2.0f * M_PI;

        Eigen::Vector3f result = fused_mean;
        result[2] = theta;

        return {result, fused_precision};
    }

    // =====================================================================
    //  Exact 3×3 Hessian via double-backprop
    //  H_ij = ∂²L/∂p_i∂p_j computed by differentiating through the gradient
    // =====================================================================
    Eigen::Matrix3f RoomConcept::autograd_hessian_3x3(const torch::Tensor& loss,
                                                       const torch::Tensor& param)
    {
        // First-order gradient with graph retained for second pass
        auto grad = torch::autograd::grad({loss}, {param},
            /*grad_outputs=*/{}, /*retain_graph=*/true, /*create_graph=*/true)[0];

        Eigen::Matrix3f H;
        for (int i = 0; i < 3; i++)
        {
            // Differentiate grad[i] w.r.t. param → row i of the Hessian
            auto gi = grad.index({i});
            auto row = torch::autograd::grad({gi}, {param},
                /*grad_outputs=*/{}, /*retain_graph=*/(i < 2), /*create_graph=*/false)[0];
            auto row_cpu = row.to(torch::kCPU);
            auto acc = row_cpu.accessor<float, 1>();
            for (int j = 0; j < 3; j++)
                H(i, j) = acc[j];
        }
        // Symmetrise to absorb any floating-point asymmetry
        H = 0.5f * (H + H.transpose());
        return H;
    }

    // =====================================================================
    //  Sliding-Window RFE: compute_rfe_loss, slide_window, collect_window_params
    //  (Paper: "Total-Time Active Inference", Section 6.4, Algorithm 1)
    // =====================================================================

    torch::Tensor RoomConcept::compute_rfe_loss() const
    {
        if (window_.empty() || !model_)
            return torch::tensor(0.0f, torch::TensorOptions().device(get_device()));

        const auto device = get_device();
        const float inv_var = 1.0f / (params.rfe_obs_sigma * params.rfe_obs_sigma);
        const float huber_delta = params.rfe_huber_delta;

        torch::Tensor total_loss = torch::tensor(0.0f, torch::TensorOptions().device(device));

        // --- 1. Boundary prior on oldest surviving state (Eq. 28) ---
        if (boundary_prior_.valid && !window_.empty())
        {
            const auto& oldest_pose = window_.front().pose;  // [3] with grad
            const auto mu = torch::tensor({boundary_prior_.mu[0], boundary_prior_.mu[1], boundary_prior_.mu[2]},
                torch::TensorOptions().dtype(torch::kFloat32).device(device));

            // Build precision tensor from Eigen matrix
            auto prec_data = torch::zeros({3, 3}, torch::TensorOptions().dtype(torch::kFloat32).device(device));
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    prec_data[i][j] = boundary_prior_.precision(i, j);

            auto raw_diff = oldest_pose - mu;
            // Wrap theta component to [-π, π] (differentiable)
            auto angle_diff = raw_diff.index({2});
            auto wrapped_diff = torch::cat({raw_diff.index({torch::indexing::Slice(0, 2)}),
                                             torch::atan2(torch::sin(angle_diff), torch::cos(angle_diff)).unsqueeze(0)});
            auto diff = wrapped_diff.unsqueeze(1);  // [3,1]
            auto boundary_loss = 0.5f * torch::matmul(diff.t(), torch::matmul(prec_data, diff)).squeeze();
            total_loss = total_loss + boundary_loss;
        }

        // --- 2. Observation factors: SDF likelihood at each timestep (Eq. 36 / Formulation B) ---
        for (const auto& slot : window_)
        {
            auto pose_xy = slot.pose.index({torch::indexing::Slice(0, 2)});    // [2]
            auto pose_theta = slot.pose.index({torch::indexing::Slice(2, 3)});  // [1]

            const auto sdf_vals = model_->sdf_at_pose(slot.lidar_points, pose_xy, pose_theta);

            const auto obs_huber = torch::nn::functional::huber_loss(
                sdf_vals,
                torch::zeros_like(sdf_vals),
                torch::nn::functional::HuberLossFuncOptions().reduction(torch::kMean).delta(huber_delta)
            );
            total_loss = total_loss + 0.5f * inv_var * obs_huber;
        }

        // --- 3. Motion factors between consecutive slots (Eq. 27, ℓ_dyn) ---
        for (size_t i = 1; i < window_.size(); i++)
        {
            const auto& curr = window_[i];
            const auto& prev = window_[i - 1];

            // Predicted delta from odometry
            auto odom_delta = torch::tensor(
                {curr.odometry_delta[0], curr.odometry_delta[1], curr.odometry_delta[2]},
                torch::TensorOptions().dtype(torch::kFloat32).device(device));

            // Actual delta from the pose parameters (differentiable)
            auto pose_delta = curr.pose - prev.pose;

            // Handle angle wrapping for the theta component
            // residual = actual_delta - odom_delta
            auto raw_residual = pose_delta - odom_delta;

            // Wrap the angular residual to [-π, π] (differentiable, no in-place ops)
            auto angle_res = raw_residual.index({2});
            auto wrapped_angle = torch::atan2(torch::sin(angle_res), torch::cos(angle_res));
            auto residual = torch::cat({raw_residual.index({torch::indexing::Slice(0, 2)}),
                                        wrapped_angle.unsqueeze(0)});

            // Build precision tensor from motion covariance
            Eigen::Matrix3f motion_prec = curr.motion_cov.inverse();
            auto prec_data = torch::zeros({3, 3}, torch::TensorOptions().dtype(torch::kFloat32).device(device));
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    prec_data[r][c] = motion_prec(r, c);

            auto res_col = residual.unsqueeze(1);  // [3,1]
            auto motion_loss = 0.5f * torch::matmul(res_col.t(), torch::matmul(prec_data, res_col)).squeeze();
            total_loss = total_loss + motion_loss;
        }

        return total_loss;
    }

    void RoomConcept::slide_window_boundary_prior_only()
    {
        if (window_.empty())
            return;

        // Compute boundary prior from the oldest slot BEFORE dropping it
        // (Paper Eq. 28: μ_∂ = MAP value, Σ_∂ = diag(∇²L)^{-1})
        const auto& oldest = window_.front();

        // Extract MAP pose (detached)
        auto pose_cpu = oldest.pose.detach().to(torch::kCPU);
        auto pose_acc = pose_cpu.accessor<float, 1>();
        boundary_prior_.mu = Eigen::Vector3f(pose_acc[0], pose_acc[1], pose_acc[2]);

        // Compute diagonal Hessian of the full RFE loss at the oldest pose
        // We approximate using the gradient magnitude: H_ii ≈ |∂L/∂θ_i| * scale
        {
            auto oldest_pose_for_hess = oldest.pose.clone().detach().requires_grad_(true);

            // Build a mini-loss involving only the oldest slot's observation
            // and its motion factor to the next slot (if any)
            auto pose_xy = oldest_pose_for_hess.index({torch::indexing::Slice(0, 2)});
            auto pose_theta = oldest_pose_for_hess.index({torch::indexing::Slice(2, 3)});

            const float inv_var = 1.0f / (params.rfe_obs_sigma * params.rfe_obs_sigma);
            auto sdf_vals = model_->sdf_at_pose(oldest.lidar_points, pose_xy, pose_theta);
            auto loss = 0.5f * inv_var * torch::nn::functional::huber_loss(
                sdf_vals, torch::zeros_like(sdf_vals),
                torch::nn::functional::HuberLossFuncOptions().reduction(torch::kMean).delta(params.rfe_huber_delta));

            // Add motion factor to next slot if it exists
            if (window_.size() > 1)
            {
                const auto& next = window_[1];
                auto next_pose = next.pose.detach();  // fixed for Hessian computation
                auto delta = next_pose - oldest_pose_for_hess;
                auto odom = torch::tensor(
                    {next.odometry_delta[0], next.odometry_delta[1], next.odometry_delta[2]},
                    torch::TensorOptions().dtype(torch::kFloat32).device(get_device()));
                auto raw_res = delta - odom;
                // Wrap theta component to [-π, π]
                auto angle_r = raw_res.index({2});
                auto residual = torch::cat({raw_res.index({torch::indexing::Slice(0, 2)}),
                                            torch::atan2(torch::sin(angle_r), torch::cos(angle_r)).unsqueeze(0)});
                Eigen::Matrix3f prec = next.motion_cov.inverse();
                auto prec_t = torch::zeros({3, 3}, torch::TensorOptions().dtype(torch::kFloat32).device(get_device()));
                for (int r = 0; r < 3; r++)
                    for (int c = 0; c < 3; c++)
                        prec_t[r][c] = prec(r, c);
                auto res_col = residual.unsqueeze(1);
                loss = loss + 0.5f * torch::matmul(res_col.t(), torch::matmul(prec_t, res_col)).squeeze();
            }

            // Add existing boundary prior if any
            if (boundary_prior_.valid)
            {
                auto mu_t = torch::tensor(
                    {boundary_prior_.mu[0], boundary_prior_.mu[1], boundary_prior_.mu[2]},
                    torch::TensorOptions().dtype(torch::kFloat32).device(get_device()));
                auto diff = (oldest_pose_for_hess - mu_t).unsqueeze(1);
                auto prec_t = torch::zeros({3, 3}, torch::TensorOptions().dtype(torch::kFloat32).device(get_device()));
                for (int r = 0; r < 3; r++)
                    for (int c = 0; c < 3; c++)
                        prec_t[r][c] = boundary_prior_.precision(r, c);
                loss = loss + 0.5f * torch::matmul(diff.t(), torch::matmul(prec_t, diff)).squeeze();
            }

            // Compute exact 3×3 Hessian via double-backprop
            Eigen::Matrix3f H = autograd_hessian_3x3(loss, oldest_pose_for_hess);

            // Clamp eigenvalues to ensure positive-definiteness
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(H);
            Eigen::Vector3f evals = eig.eigenvalues().cwiseMax(1e-3f);
            H = eig.eigenvectors() * evals.asDiagonal() * eig.eigenvectors().transpose();

            boundary_prior_.precision = H;
        }

        boundary_prior_.valid = true;

        // NOTE: pop_front is NOT done here — the caller (cheap slide in update())
        // already removed the slot before Adam ran.
    }

    std::vector<torch::Tensor> RoomConcept::collect_window_params() const
    {
        std::vector<torch::Tensor> params;
        params.reserve(window_.size());
        for (const auto& slot : window_)
            params.push_back(slot.pose);
        return params;
    }

} // namespace rc
