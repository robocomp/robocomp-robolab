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

        const auto& [gt_, lidar_high_] = run_ctx_.sensor_buffer->read_last();
        if (!lidar_high_.has_value() || lidar_high_->first.empty())
            return false;

        const auto& pts = lidar_high_->first;
        Eigen::Vector2f init_xy = Eigen::Vector2f::Zero();
        float init_phi = 0.f;
        bool have_saved_pose = false;

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

        if (!have_saved_pose)
        {
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

            // ===== 4. SNAPSHOT VELOCITY & ODOMETRY HISTORY =====
            auto vel_snap  = run_ctx_.velocity_buffer->get_snapshot<0>();
            auto odom_snap = run_ctx_.odometry_buffer->get_snapshot<0>();

            // ===== 5. RUN LOCALIZATION UPDATE =====
            const auto res = update(lidar_high_.value(), vel_snap, odom_snap);

            // ===== 6. PUBLISH RESULT =====
            {
                std::lock_guard lock(result_mutex_);
                last_result_ = res;
            }
            if (res.ok && !loc_initialized_.load())
                loc_initialized_ = true;

            // Adaptive pacing: slow down on repeated timestamps, speed up on new data
            const auto current_ts = lidar_high_->second;
            if (current_ts == last_ts)
                wait_period = std::min(wait_period + std::chrono::milliseconds(1), kMaxWait);
            else
                wait_period = std::max(wait_period - std::chrono::milliseconds(1), kMinWait);

            last_ts = current_ts;
            std::this_thread::sleep_for(wait_period);
        }

        loc_running_ = false;
        qInfo() << "[LocThread] Localization thread stopped";
    }

      // Static initialization: limit PyTorch threads to avoid CPU overload
    static bool torch_threads_initialized = []() {
        torch::set_num_threads(2);  // Limit to 2 threads
        torch::set_num_interop_threads(1);
        return true;
    }();

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

        // Get velocities from odometry prior
        const float linear_speed = odometry_prior.delta_pose.head<2>().norm() /
                                   std::max(odometry_prior.dt, 0.001f);
        const float angular_speed = std::abs(odometry_prior.delta_pose[2]) /
                                    std::max(odometry_prior.dt, 0.001f);

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

        // ===== MINIMISATION STEP (Minimize Variational Free Energy) =====
        // Subsample lidar points for speed
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

        // ===== PREDICTION-BASED EARLY EXIT =====
        // If the predicted pose already has low SDF error, skip optimization entirely.
        // This saves significant CPU when the robot moves smoothly.
        if (params.prediction_early_exit &&
            last_update_result.ok &&
            odometry_prior.valid &&
            tracking_step_count_ > params.min_tracking_steps)
        {
            // Check pose uncertainty (trace of position covariance)
            const float pose_uncertainty = current_covariance(0,0) + current_covariance(1,1);

            if (pose_uncertainty < params.max_uncertainty_for_early_exit)
            {
                // Evaluate SDF at predicted pose (no gradients needed)
                torch::NoGradGuard no_grad;
                const auto sdf_pred = model_->sdf(points_tensor);
                const float mean_sdf_pred = torch::mean(torch::abs(sdf_pred)).item<float>();

                // Early exit threshold
                const float prediction_trust_threshold = params.sigma_sdf * params.prediction_trust_factor;

                if (mean_sdf_pred < prediction_trust_threshold)
                {
                    // Prediction is good enough - skip optimization entirely
                    prediction_early_exits_++;

                    res.ok = true;
                    res.final_loss = mean_sdf_pred;
                    res.sdf_mse = mean_sdf_pred;
                    res.iterations_used = 0;
                    res.state = model_->get_state();

                    // Build pose from predicted values (already set in model)
                    const float x = res.state[2];
                    const float y = res.state[3];
                    const float phi = res.state[4];

                    // Apply smoothing even for early exit
                    float smoothed_x = x, smoothed_y = y, smoothed_phi = phi;
                    if (has_smoothed_pose_)
                    {
                        const float alpha = params.pose_smoothing;
                        smoothed_x = alpha * smoothed_pose_[0] + (1.0f - alpha) * x;
                        smoothed_y = alpha * smoothed_pose_[1] + (1.0f - alpha) * y;
                        float angle_diff = phi - smoothed_pose_[2];
                        while (angle_diff > M_PI) angle_diff -= 2.0f * M_PI;
                        while (angle_diff < -M_PI) angle_diff += 2.0f * M_PI;
                        smoothed_phi = smoothed_pose_[2] + (1.0f - alpha) * angle_diff;
                    }
                    smoothed_pose_ = Eigen::Vector3f(smoothed_x, smoothed_y, smoothed_phi);
                    has_smoothed_pose_ = true;

                    res.state[2] = smoothed_x;
                    res.state[3] = smoothed_y;
                    res.state[4] = smoothed_phi;

                    Eigen::Affine2f pose = Eigen::Affine2f::Identity();
                    pose.translation() = Eigen::Vector2f{smoothed_x, smoothed_y};
                    pose.linear() = Eigen::Rotation2Df(smoothed_phi).toRotationMatrix();
                    res.robot_pose = pose;

                    // Use PROPAGATED covariance (uncertainty grows when we skip optimization)
                    // current_covariance was already updated with prediction.propagated_cov above
                    res.covariance = current_covariance;

                    // Compute innovation even on early exit: smoothed pose vs prediction
                    if (model_->has_prediction)
                    {
                        res.innovation[0] = smoothed_x - model_->predicted_pos[0].item<float>();
                        res.innovation[1] = smoothed_y - model_->predicted_pos[1].item<float>();
                        float p_theta = model_->predicted_theta[0].item<float>();
                        res.innovation[2] = smoothed_phi - p_theta;
                        while (res.innovation[2] > M_PI) res.innovation[2] -= 2.0f * M_PI;
                        while (res.innovation[2] < -M_PI) res.innovation[2] += 2.0f * M_PI;
                        res.innovation_norm = std::sqrt(res.innovation[0]*res.innovation[0] +
                                                        res.innovation[1]*res.innovation[1]);
                    }
                    else
                    {
                        res.innovation = Eigen::Vector3f::Zero();
                        res.innovation_norm = 0.0f;
                    }

                    // Update current_covariance to reflect the propagated uncertainty
                    // (This is already done above in the prediction step, but we store it in result)

                    // Update model with smoothed pose
                    model_->robot_pos.data().copy_(torch::tensor({smoothed_x, smoothed_y},
                        torch::TensorOptions().device(get_device())));
                    model_->robot_theta.data().copy_(torch::tensor({smoothed_phi},
                        torch::TensorOptions().device(get_device())));
                    model_->robot_prev_pose = res.robot_pose;
                    model_->has_prediction = false;

                    last_update_result = res;
                    last_lidar_timestamp = lidar.second;

                    // Debug: log early exits periodically
                    if (prediction_early_exits_ % 50 == 0)
                    {
                        qDebug() << "[EARLY_EXIT] Skipped optimization. SDF:" << mean_sdf_pred
                                 << "m, threshold:" << prediction_trust_threshold
                                 << "m, total early exits:" << prediction_early_exits_;
                    }

                    return res;
                }
            }
        }

        // Use different learning rates for position vs rotation
        std::vector<torch::optim::OptimizerParamGroup> param_groups;
        param_groups.emplace_back(
            std::vector<torch::Tensor>{model_->robot_pos},
            std::make_unique<torch::optim::AdamOptions>(params.learning_rate_pos)
        );
        param_groups.emplace_back(
            std::vector<torch::Tensor>{model_->robot_theta},
            std::make_unique<torch::optim::AdamOptions>(params.learning_rate_rot)
        );
        torch::optim::Adam optimizer(param_groups);

        // Compute velocity-adaptive weights for gradient scaling
        const Eigen::Vector3f velocity_weights = compute_velocity_adaptive_weights(odometry_prior);

        // Determine number of iterations
        const int max_iters = params.num_iterations;

        float last_loss = std::numeric_limits<float>::infinity();
        float prev_loss = std::numeric_limits<float>::infinity();
        int iterations = 0;
        for(int i=0; i < max_iters; ++i)
        {
            optimizer.zero_grad();
            const torch::Tensor loss = loss_sdf_mse(points_tensor, *model_);
            loss.backward();

            // Apply velocity-adaptive gradient weighting
            // This emphasizes parameters that are expected to change based on motion
            {
                torch::NoGradGuard no_grad;
                if (model_->robot_pos.grad().defined())
                {
                    model_->robot_pos.mutable_grad().index({0}) *= velocity_weights[0];  // x weight
                    model_->robot_pos.mutable_grad().index({1}) *= velocity_weights[1];  // y weight
                }
                if (model_->robot_theta.grad().defined())
                {
                    model_->robot_theta.mutable_grad() *= velocity_weights[2];   // theta weight
                }
            }

            optimizer.step();

            prev_loss = last_loss;
            last_loss = loss.item<float>();
            iterations = i + 1;

            // Early exit: absolute threshold or relative convergence
            if(last_loss < params.min_loss_threshold)
                break;
            // If loss changed by less than 1%, we've converged
            if (i > 5 && std::abs(prev_loss - last_loss) < 0.01f * prev_loss)
                break;
        }

        // ===== COVARIANCE UPDATE =====
        // Active Inference / EKF update: P_posterior = (P_prior^-1 + H_likelihood)^-1
        // Using diagonal Hessian approximation for efficiency (Gauss-Newton style)
        try {
            // Compute gradients for diagonal Hessian approximation
            const torch::Tensor sdf_vals = model_->sdf(points_tensor);
            constexpr float huber_delta = 0.15f;
            const torch::Tensor likelihood_loss = torch::nn::functional::huber_loss(
                sdf_vals,
                torch::zeros_like(sdf_vals),
                torch::nn::functional::HuberLossFuncOptions().reduction(torch::kMean).delta(huber_delta)
            );

            // Get parameters
            std::vector<torch::Tensor> params_list = model_->optim_parameters();

            // Compute first-order gradients
            auto grads = torch::autograd::grad({likelihood_loss}, params_list,
                                               /*grad_outputs=*/{},
                                               /*retain_graph=*/false,
                                               /*create_graph=*/false);

            // Diagonal Hessian approximation: H_ii ≈ (∂L/∂θ_i)² / L
            // This is a Fisher Information approximation
            torch::Tensor grad_flat = torch::cat({grads[0].flatten(), grads[1].flatten()}, 0);
            float loss_val = likelihood_loss.item<float>();

            // Compute diagonal Hessian approximation (must be on CPU for accessor)
            Eigen::Matrix3f H_likelihood = Eigen::Matrix3f::Zero();
            auto grad_cpu = grad_flat.to(torch::kCPU);
            auto g_acc = grad_cpu.accessor<float, 1>();

            // Use gradient magnitude as proxy for curvature
            // Scale by number of points for proper normalization
            float scale = static_cast<float>(points_tensor.size(0)) / (loss_val + 1e-6f);
            for (int i = 0; i < 3; i++) {
                H_likelihood(i, i) = std::abs(g_acc[i]) * scale + 1e-4f;  // Ensure positive
            }

            // ===== BAYESIAN FUSION: P_posterior = (P_prior^-1 + H_likelihood)^-1 =====
            Eigen::Matrix3f prior_precision = current_covariance.inverse();

            // Posterior precision = prior precision + likelihood precision (Hessian)
            // Add small regularization for numerical stability
            constexpr float lambda = 1e-4f;
            Eigen::Matrix3f posterior_precision = prior_precision + H_likelihood
                                                 + lambda * Eigen::Matrix3f::Identity();

            // Compute posterior covariance
            Eigen::Matrix3f new_cov = posterior_precision.inverse();

            // Compute condition number for diagnostics
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(posterior_precision);
            const auto eigenvalues = solver.eigenvalues();
            const float max_ev = eigenvalues.maxCoeff();
            const float min_ev = eigenvalues.minCoeff();
            res.condition_number = (min_ev > 1e-8f) ? (max_ev / min_ev) : 1e8f;

            // Validate covariance
            if (new_cov.allFinite() && new_cov.determinant() > 1e-10f && res.condition_number < 1e6f) {
                current_covariance = new_cov;
                res.covariance = new_cov;
            } else {
                // Keep propagated covariance if fusion fails
                res.covariance = current_covariance;
            }

        } catch (const std::exception &e) {
            // Hessian computation failed, use propagated covariance
            std::cerr << "Covariance update failed: " << e.what() << std::endl;
            res.covariance = current_covariance;
            res.condition_number = -1.0f; // Signal failure
        }

        res.ok = true;
        res.final_loss = last_loss;
        res.sdf_mse = compute_sdf_mse_unscaled(points_tensor, *model_);  // Unscaled for UI
        res.iterations_used = iterations;
        res.state = model_->get_state();
        {
            float x = res.state[2];
            float y = res.state[3];
            float phi = res.state[4];

            // ===== POSE SMOOTHING to reduce jitter =====
            // Apply exponential moving average filter
            if (has_smoothed_pose_)
            {
                const float alpha = params.pose_smoothing;

                x = alpha * smoothed_pose_[0] + (1.0f - alpha) * x;
                y = alpha * smoothed_pose_[1] + (1.0f - alpha) * y;

                // Smooth angle carefully (handle wrap-around)
                float angle_diff = phi - smoothed_pose_[2];
                while (angle_diff > M_PI) angle_diff -= 2.0f * M_PI;
                while (angle_diff < -M_PI) angle_diff += 2.0f * M_PI;
                phi = smoothed_pose_[2] + (1.0f - alpha) * angle_diff;
                while (phi > M_PI) phi -= 2.0f * M_PI;
                while (phi < -M_PI) phi += 2.0f * M_PI;
            }

            // Store smoothed pose for next iteration
            smoothed_pose_ = Eigen::Vector3f(x, y, phi);
            has_smoothed_pose_ = true;


            // Update model with smoothed values
            model_->robot_pos.data().copy_(torch::tensor({x, y},
                torch::TensorOptions().device(get_device())));
            model_->robot_theta.data().copy_(torch::tensor({phi},
                torch::TensorOptions().device(get_device())));

            // Update result state with smoothed values
            res.state[2] = x;
            res.state[3] = y;
            res.state[4] = phi;

            Eigen::Affine2f pose = Eigen::Affine2f::Identity();
            pose.translation() = Eigen::Vector2f{x, y};
            pose.linear() = Eigen::Rotation2Df(phi).toRotationMatrix();
            res.robot_pose = pose;

            // ===== COMPUTE INNOVATION (Kalman residual) =====
            // Innovation = optimized_state - predicted_state
            if (model_->has_prediction)
            {
                res.innovation[0] = x - model_->predicted_pos[0].item<float>();
                res.innovation[1] = y - model_->predicted_pos[1].item<float>();
                float pred_theta = model_->predicted_theta[0].item<float>();
                res.innovation[2] = phi - pred_theta;
                // Normalize angle difference to [-pi, pi]
                while (res.innovation[2] > M_PI) res.innovation[2] -= 2.0f * M_PI;
                while (res.innovation[2] < -M_PI) res.innovation[2] += 2.0f * M_PI;

                // Compute norm (position only for simplicity)
                res.innovation_norm = std::sqrt(res.innovation[0]*res.innovation[0] +
                                                res.innovation[1]*res.innovation[1]);
            }
        }

        // ===== UPDATE MODEL STATE FOR NEXT ITERATION =====
        // Store current pose for next prediction step
        model_->robot_prev_pose = res.robot_pose;

        // Store current covariance as tensor for next prediction
        // Create on CPU first for accessor, then move to device
        auto cov_cpu = torch::zeros({3, 3}, torch::kFloat32);
        auto cov_acc = cov_cpu.accessor<float, 2>();
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                cov_acc[i][j] = res.covariance(i, j);
        model_->prev_cov = cov_cpu.to(get_device());

        // Reset prediction flag for next cycle
        model_->has_prediction = false;

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

} // namespace rc
