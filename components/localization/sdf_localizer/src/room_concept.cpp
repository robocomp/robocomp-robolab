#include "room_concept.h"
#include "pointcloud_center_estimator.h"

#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <sys/stat.h>
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
        if (loc_running_.load() || loc_thread_.joinable()) return;
        stop_requested_ = false;

        // If CUDA is requested, initialize the CUDA context HERE on the calling thread
        // (main/Qt thread) before spawning the localization thread.
        // libtorch requires that CUDA be initialized on the thread that will own the
        // context, or at minimum that the CUDA dispatcher has been activated before
        // worker threads try to create CUDA tensors.  Without this, is_available()
        // returns true (driver found) but CUDA kernels fail to register.
        if (params.use_cuda && torch::cuda::is_available())
        {
            try {
                // Warm-up: create and discard a tiny CUDA tensor to trigger CUDA init
                auto tmp = torch::zeros({1}, torch::TensorOptions().device(torch::kCUDA));
                (void)tmp;
                qInfo() << "[RoomConcept] CUDA initialized successfully on calling thread.";
            } catch (const std::exception& e) {
                qWarning() << "[RoomConcept] CUDA init failed:" << e.what()
                           << "— falling back to CPU.";
                params.use_cuda = false;
            }
        }

        loc_running_ = true;
        loc_thread_ = std::thread(&RoomConcept::run, this);
    }

    void RoomConcept::stop()
    {
        stop_requested_ = true;
        if (loc_thread_.joinable())
            loc_thread_.join();
        loc_running_ = false;
    }

    void RoomConcept::init_debug_log()
    {
        if (!params.debug_log_enabled) return;

        ::mkdir("tmp", 0755);
        ::mkdir("tmp/sdf_localizer", 0755);

        const std::string path = "tmp/sdf_localizer/log.csv";
        debug_log_.open(path, std::ios::out | std::ios::trunc);
        if (!debug_log_.is_open())
        {
            std::cerr << "[DebugLog] ERROR: could not open " << path << std::endl;
            return;
        }

        // CSV header — one row per update() call
        debug_log_
            << "ts_ms"
            << ",wall_ms"
            << ",dt_ms"
            << ",n_lidar"
            << ",vel_adv_y"
            << ",vel_rot"
            << ",odom_adv"
            << ",odom_rot"
            << ",cmd_valid"
            << ",cmd_dx,cmd_dy,cmd_dth"
            << ",cmd_cov_xx,cmd_cov_tt"
            << ",meas_valid"
            << ",meas_dx,meas_dy,meas_dth"
            << ",meas_cov_xx,meas_cov_tt"
            << ",pred_x,pred_y,pred_theta"
            << ",slot_mcov_xx,slot_mcov_tt"
            << ",early_exit"
            << ",iters"
            << ",final_loss"
            << ",lr_eff"
            << ",res_x,res_y,res_theta"
            << ",innov_x,innov_y,innov_theta"
            << ",innov_norm"
            << ",sdf_mse"
            << ",cov_xx,cov_tt"
            << ",cond_num"
            << ",window_size"
            << ",tracking_steps"
            << ",loss_boundary"
            << ",loss_obs"
            << ",loss_motion"
            << ",loss_corner"
            << ",loss_init"
            << ",t_update_ms"
            << ",t_adam_ms"
            << ",t_cov_ms"
            << ",t_breakdown_ms"
            << "\n";
        debug_log_.flush();
        std::cout << "[DebugLog] Logging to " << path << std::endl;
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
            const auto& [gt_, lidar_high_, obstacles_] = run_ctx_.sensor_buffer->read_last();
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
        init_debug_log();
        rerun_frame_counter_ = 0;

        if (params.rerun_enabled)
        {
            RerunLogger::Config cfg;
            cfg.enabled = true;
            cfg.host = params.rerun_host;
            cfg.port = params.rerun_port;
            cfg.sdf_every_n = params.rerun_sdf_every_n;
            cfg.sdf_resolution = params.rerun_sdf_resolution;
            cfg.max_queue = params.rerun_max_queue;
            rerun_logger_.init(cfg);
        }

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
            const auto& [gt_, lidar_high_, obstacles_] = run_ctx_.sensor_buffer->read_last();
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
            const auto t_update_start_ = std::chrono::high_resolution_clock::now();
            const auto res = update(lidar_high_.value(), vel_snap, odom_snap);
            const float update_ms = std::chrono::duration<float, std::milli>(
                std::chrono::high_resolution_clock::now() - t_update_start_).count();

            if (params.rerun_enabled)
            {
                RerunFrame rf;
                rf.ts_ms = res.timestamp_ms;
                rf.x = res.state[2];
                rf.y = res.state[3];
                rf.theta = res.state[4];
                rf.innov_x = res.innovation[0];
                rf.innov_y = res.innovation[1];
                rf.innov_theta = res.innovation[2];
                rf.pred_x = rf.x - rf.innov_x;
                rf.pred_y = rf.y - rf.innov_y;
                rf.pred_theta = rf.theta - rf.innov_theta;
                rf.early_exit = (res.iterations_used <= 0);
                rf.window_size = static_cast<int>(window_mgr_.size());
                rf.iters = res.iterations_used;

                rf.loss_init = last_loss_init_;
                rf.final_loss = res.final_loss;
                rf.loss_boundary = last_loss_breakdown_.boundary;
                rf.loss_obs = last_loss_breakdown_.obs;
                rf.loss_motion = last_loss_breakdown_.motion;
                rf.loss_corner = last_loss_breakdown_.corner;

                rf.sdf_mse = res.sdf_mse;
                rf.innov_norm = res.innovation_norm;
                rf.cov_xx = res.covariance(0, 0);
                rf.cov_xy = res.covariance(0, 1);
                rf.cov_yy = res.covariance(1, 1);
                rf.cov_tt = res.covariance(2, 2);
                rf.cond_num = res.condition_number;

                rf.t_update_ms = update_ms;
                rf.t_adam_ms = last_t_adam_ms_;
                rf.t_cov_ms = last_t_cov_ms_;
                rf.t_breakdown_ms = last_t_breakdown_ms_;

                // Send real room polygon once so the bridge can draw corner-to-corner contour.
                if (!rerun_room_polygon_sent_ && model_ != nullptr && model_->use_polygon && model_->polygon_vertices.defined())
                {
                    auto poly_cpu = model_->polygon_vertices.to(torch::kCPU);
                    if (poly_cpu.dim() == 2 && poly_cpu.size(1) == 2 && poly_cpu.size(0) >= 3)
                    {
                        auto acc = poly_cpu.accessor<float, 2>();
                        rf.has_room_polygon = true;
                        rf.room_polygon.reserve(static_cast<size_t>(poly_cpu.size(0)));
                        for (int i = 0; i < poly_cpu.size(0); ++i)
                            rf.room_polygon.push_back({acc[i][0], acc[i][1]});
                        rerun_room_polygon_sent_ = true;
                    }
                }

                rf.lidar_points.reserve(res.lidar_scan.size());
                for (const auto &p : res.lidar_scan)
                    rf.lidar_points.push_back({p.x(), p.y(), p.z()});

                // Optional SDF grid every N frames.
                rerun_frame_counter_++;
                const int sdf_every_n = std::max(0, params.rerun_sdf_every_n);
                const bool send_sdf = (sdf_every_n > 0) && (rerun_frame_counter_ % sdf_every_n == 0) && (model_ != nullptr);
                if (send_sdf)
                {
                    const int res_grid = std::max(8, params.rerun_sdf_resolution);
                    const float half_w = 0.5f * std::max(0.1f, res.state[0]);
                    const float half_h = 0.5f * std::max(0.1f, res.state[1]);
                    const float span = 2.f * std::max(half_w, half_h);
                    const float cell = span / static_cast<float>(std::max(1, res_grid - 1));
                    const float ox = -0.5f * span;
                    const float oy = -0.5f * span;

                    std::vector<Eigen::Vector3f> grid_robot;
                    grid_robot.reserve(static_cast<size_t>(res_grid) * static_cast<size_t>(res_grid));

                    const float x = res.state[2];
                    const float y = res.state[3];
                    const float th = res.state[4];
                    const float c = std::cos(th);
                    const float s = std::sin(th);

                    for (int iy = 0; iy < res_grid; ++iy)
                    {
                        const float wy = oy + static_cast<float>(iy) * cell;
                        for (int ix = 0; ix < res_grid; ++ix)
                        {
                            const float wx = ox + static_cast<float>(ix) * cell;
                            const float dx = wx - x;
                            const float dy = wy - y;
                            const float rx = c * dx + s * dy;
                            const float ry = -s * dx + c * dy;
                            grid_robot.emplace_back(rx, ry, 0.f);
                        }
                    }

                    torch::NoGradGuard no_grad;
                    auto grid_t = points_to_tensor_xyz(grid_robot, get_device());
                    auto sdf_t = model_->sdf(grid_t).to(torch::kCPU);
                    auto acc = sdf_t.accessor<float, 1>();

                    rf.has_sdf_grid = true;
                    rf.sdf_w = res_grid;
                    rf.sdf_h = res_grid;
                    rf.sdf_origin_x = ox;
                    rf.sdf_origin_y = oy;
                    rf.sdf_cell_size = cell;
                    rf.sdf_values.resize(static_cast<size_t>(res_grid) * static_cast<size_t>(res_grid));
                    for (size_t i = 0; i < rf.sdf_values.size(); ++i)
                        rf.sdf_values[i] = acc[i];
                }

                rerun_logger_.log_frame(std::move(rf));
            }

            // ===== 7. PUBLISH RESULT =====
            {
                std::lock_guard lock(result_mutex_);
                last_result_ = res;
            }
            if (res.ok && !loc_initialized_.load())
                loc_initialized_ = true;

            // ===== 8. RECOVERY DETECTION =====
            {
                const float avg_sdf_err = std::sqrt(res.sdf_mse);
                if (recovery_.check(avg_sdf_err, res.iterations_used,
                                    params.recovery_loss_threshold, params.recovery_consecutive_count))
                {
                    qWarning() << "[LocThread] Recovery triggered after" << recovery_.consecutive_bad_frames
                               << "bad frames. avg_sdf_err=" << avg_sdf_err << "m"
                               << "Running grid search...";
                    const auto& pts = lidar_high_->first;
                    grid_search_initial_pose(pts, 0.5f, static_cast<float>(M_PI_4));
                    window_mgr_.clear();
                    recovery_.on_recovery_done(params.recovery_cooldown_frames);
                    qInfo() << "[LocThread] Recovery complete.";
                }
            }

            last_ts = current_ts;
            // Adaptive polling: throttle when stationary + early-exiting (pose is good),
            // snap back to fast polling when Adam runs (motion or correction needed).
            if (res.iterations_used > 0)
                wait_period = kMinWait;   // motion or correction needed → full rate
            else
                wait_period = std::min(wait_period + std::chrono::milliseconds(2), kMaxWait);
            update_ms_accum_ += update_ms;
            update_ms_count_++;

            // ===== DIFFERENTIAL TEST: RFE vs single-step vs prediction-only =====
            // Compares SDF accuracy AND pose jitter (temporal consistency).
            // Single-step will typically achieve lower SDF (it's unconstrained), but
            // RFE should show lower jitter (the window regularises across time).
            // Enable via params.differential_test_enabled.
            if (params.differential_test_enabled && res.ok)
            {
                const bool adam_ran = res.iterations_used > 0;
                const bool sample_early_exit = !adam_ran && (diff_test_.early_exit_seen++ % 50 == 0);

                if (adam_ran || sample_early_exit)
                {
                    // Recover (or use) the predicted pose
                    float pred_x, pred_y, pred_theta;
                    if (adam_ran)
                    {
                        pred_x     = res.state[2] - res.innovation[0];
                        pred_y     = res.state[3] - res.innovation[1];
                        pred_theta = res.state[4] - res.innovation[2];
                    }
                    else
                    {
                        pred_x     = res.state[2];
                        pred_y     = res.state[3];
                        pred_theta = res.state[4];
                    }

                    const auto& pts = lidar_high_->first;
                    auto pts_tensor = points_to_tensor_xyz(pts, get_device());

                    // 1. Prediction-only SDF
                    float pred_sdf;
                    {
                        torch::NoGradGuard no_grad;
                        auto xy = torch::tensor({pred_x, pred_y},
                            torch::TensorOptions().dtype(torch::kFloat32).device(get_device()));
                        auto th = torch::tensor({pred_theta},
                            torch::TensorOptions().dtype(torch::kFloat32).device(get_device()));
                        auto sdf_vals = model_->sdf_at_pose(pts_tensor, xy, th);
                        pred_sdf = torch::median(torch::abs(sdf_vals)).item<float>();
                    }

                    // 2. Single-step Adam (no window, no motion factors) — returns pose + SDF
                    const float rfe_sdf = res.sdf_mse;
                    const float rfe_x = res.state[2], rfe_y = res.state[3], rfe_theta = res.state[4];

                    auto single_pose = torch::tensor({pred_x, pred_y, pred_theta},
                        torch::TensorOptions().dtype(torch::kFloat32).device(get_device())).requires_grad_(true);
                    {
                        const float inv_var = 1.0f / (params.rfe_obs_sigma * params.rfe_obs_sigma);
                        torch::optim::Adam opt({torch::optim::OptimizerParamGroup({single_pose},
                            std::make_unique<torch::optim::AdamOptions>(params.learning_rate_pos))});
                        for (int i = 0; i < params.num_iterations; ++i)
                        {
                            opt.zero_grad();
                            auto xy = single_pose.index({torch::indexing::Slice(0, 2)});
                            auto th = single_pose.index({torch::indexing::Slice(2, 3)});
                            auto loss = 0.5f * inv_var * torch::nn::functional::huber_loss(
                                model_->sdf_at_pose(pts_tensor, xy, th),
                                torch::zeros({pts_tensor.size(0)}, pts_tensor.options()),
                                torch::nn::functional::HuberLossFuncOptions().reduction(torch::kMean)
                                    .delta(params.rfe_huber_delta));
                            loss.backward();
                            opt.step();
                        }
                    }
                    float single_sdf;
                    {
                        torch::NoGradGuard no_grad;
                        auto xy = single_pose.index({torch::indexing::Slice(0, 2)});
                        auto th = single_pose.index({torch::indexing::Slice(2, 3)});
                        auto sdf_vals = model_->sdf_at_pose(pts_tensor, xy, th);
                        single_sdf = torch::median(torch::abs(sdf_vals)).item<float>();
                    }
                    auto sp_cpu = single_pose.detach().to(torch::kCPU);
                    auto sp = sp_cpu.accessor<float, 1>();
                    const float s_x = sp[0], s_y = sp[1], s_theta = sp[2];

                    // --- Correction jitter (how far each method moved from prediction) ---
                    const float rfe_corr = std::sqrt((rfe_x - pred_x) * (rfe_x - pred_x) +
                                                     (rfe_y - pred_y) * (rfe_y - pred_y));
                    const float single_corr = std::sqrt((s_x - pred_x) * (s_x - pred_x) +
                                                        (s_y - pred_y) * (s_y - pred_y));
                    auto wrap = [](float a) { while (a > M_PI) a -= 2*M_PI; while (a < -M_PI) a += 2*M_PI; return a; };
                    const float rfe_tcorr = std::abs(wrap(rfe_theta - pred_theta));
                    const float single_tcorr = std::abs(wrap(s_theta - pred_theta));

                    diff_test_.rfe_jitter_sum += rfe_corr;
                    diff_test_.single_jitter_sum += single_corr;
                    diff_test_.rfe_theta_jitter_sum += rfe_tcorr;
                    diff_test_.single_theta_jitter_sum += single_tcorr;

                    // --- Correction consistency (how stable corrections are across frames) ---
                    // correction = optimised - predicted (for each method)
                    const float rfe_cx = rfe_x - pred_x, rfe_cy = rfe_y - pred_y;
                    const float rfe_ctheta = wrap(rfe_theta - pred_theta);
                    const float single_cx = s_x - pred_x, single_cy = s_y - pred_y;
                    const float single_ctheta = wrap(s_theta - pred_theta);

                    if (diff_test_.has_prev)
                    {
                        // RFE correction change
                        const float drx = rfe_cx - diff_test_.prev_rfe_cx;
                        const float dry = rfe_cy - diff_test_.prev_rfe_cy;
                        diff_test_.rfe_corr_consistency_sum += std::sqrt(drx*drx + dry*dry);
                        diff_test_.rfe_theta_consistency_sum += std::abs(wrap(rfe_ctheta - diff_test_.prev_rfe_ctheta));

                        // Single-step correction change
                        const float dsx = single_cx - diff_test_.prev_single_cx;
                        const float dsy = single_cy - diff_test_.prev_single_cy;
                        diff_test_.single_corr_consistency_sum += std::sqrt(dsx*dsx + dsy*dsy);
                        diff_test_.single_theta_consistency_sum += std::abs(wrap(single_ctheta - diff_test_.prev_single_ctheta));
                    }
                    diff_test_.prev_rfe_cx = rfe_cx; diff_test_.prev_rfe_cy = rfe_cy; diff_test_.prev_rfe_ctheta = rfe_ctheta;
                    diff_test_.prev_single_cx = single_cx; diff_test_.prev_single_cy = single_cy; diff_test_.prev_single_ctheta = single_ctheta;
                    diff_test_.has_prev = true;

                    // --- Accumulate SDF stats ---
                    diff_test_.pred_sdf_sum   += pred_sdf;
                    diff_test_.single_sdf_sum += single_sdf;
                    diff_test_.rfe_sdf_sum    += rfe_sdf;
                    diff_test_.count++;
                    if (adam_ran) diff_test_.adam_frames++;
                    if (rfe_sdf < single_sdf - 1e-5f)      diff_test_.rfe_wins++;
                    else if (single_sdf < rfe_sdf - 1e-5f)  diff_test_.single_wins++;

                    // Periodic report every 100 frames
                    if (diff_test_.count % 100 == 0 && diff_test_.count > 0)
                    {
                        const int n = diff_test_.count;
                        std::cout << "\n===== DIFFERENTIAL TEST (" << n << " frames, "
                                  << diff_test_.adam_frames << " Adam + "
                                  << (n - diff_test_.adam_frames) << " early-exit) =====\n"
                                  << "  --- SDF accuracy (lower = better fit to room) ---\n"
                                  << "  Prediction-only  avg SDF: " << (diff_test_.pred_sdf_sum / n) << " m\n"
                                  << "  Single-step Adam avg SDF: " << (diff_test_.single_sdf_sum / n) << " m\n"
                                  << "  Full RFE (W=" << params.rfe_window_size << ")   avg SDF: "
                                  << (diff_test_.rfe_sdf_sum / n) << " m\n"
                                  << "  SDF wins — RFE: " << diff_test_.rfe_wins
                                  << "  Single: " << diff_test_.single_wins << "\n"
                                  << "  --- Correction jitter (lower = more stable) ---\n"
                                  << "  RFE    avg pos correction: " << (diff_test_.rfe_jitter_sum / n * 1000) << " mm"
                                  << "  avg θ correction: " << (diff_test_.rfe_theta_jitter_sum / n * 180 / M_PI) << " deg\n"
                                  << "  Single avg pos correction: " << (diff_test_.single_jitter_sum / n * 1000) << " mm"
                                  << "  avg θ correction: " << (diff_test_.single_theta_jitter_sum / n * 180 / M_PI) << " deg\n";
                        if (n > 1)
                        {
                            const int np = n - 1;
                            std::cout << "  --- Correction consistency (lower = smoother) ---\n"
                                      << "  RFE    avg Δcorr: " << (diff_test_.rfe_corr_consistency_sum / np * 1000) << " mm/frame"
                                      << "  avg Δθcorr: " << (diff_test_.rfe_theta_consistency_sum / np * 180 / M_PI) << " deg/frame\n"
                                      << "  Single avg Δcorr: " << (diff_test_.single_corr_consistency_sum / np * 1000) << " mm/frame"
                                      << "  avg Δθcorr: " << (diff_test_.single_theta_consistency_sum / np * 180 / M_PI) << " deg/frame\n";
                        }
                        std::cout << "===========================================\n" << std::endl;
                    }
                }
            }
            const float avg_update_ms = update_ms_accum_ / update_ms_count_;
            const int fps = loc_fps_.print("[LocThread] update_ms(last/avg)=" + std::to_string(static_cast<int>(update_ms))
                               + "/" + std::to_string(static_cast<int>(avg_update_ms)), 2000);
            if (fps > 0) { update_ms_accum_ = 0.f; update_ms_count_ = 0; }
            std::this_thread::sleep_for(wait_period);
        }

        rerun_logger_.stop();
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
        const int max_samples = params.orientation_search_max_samples;
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
        const float margin = params.grid_search_wall_margin;
        min_x += margin; max_x -= margin;
        min_y += margin; max_y -= margin;

        // Subsample lidar points for faster evaluation
        std::vector<Eigen::Vector3f> sample_points;
        const int max_samples = params.grid_search_max_samples;
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
        return best_loss < params.grid_search_good_threshold;
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
        prev_sdf_mse_ = 0.f;  // Reset boundary quality gate
        window_mgr_.clear();
        rerun_room_polygon_sent_ = false;

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
        prev_sdf_mse_ = 0.f;  // Reset boundary quality gate
        window_mgr_.clear();
        rerun_room_polygon_sent_ = false;

        // Initialize corner detector with model polygon
        corner_detector_.set_model_corners(polygon_vertices);

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
        manual_reset_frames_ = manual_reset ? params.manual_reset_skip_frames : 0;

        // Clear any previous prediction
        model_->has_prediction = false;
        model_->robot_prev_pose = std::nullopt;

        // Clear sliding window (stale poses are invalid after manual reset)
        window_mgr_.clear();

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
            w_x = params.combined_motion_weight;
            w_y = params.combined_motion_weight;
            w_theta = params.combined_motion_weight;
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
        t_update_start_ = std::chrono::high_resolution_clock::now();
        UpdateResult res;
        if (lidar.first.empty() || model_ == nullptr)
            return res;

        res.lidar_scan = lidar.first;   // store scan for synchronized visualization

        // ===== MANUAL RESET: skip optimization during settle period =====
        if (manual_reset_frames_ > 0)
        {
            manual_reset_frames_--;
            const auto state = model_->get_state();
            res.ok = true;
            res.state = state;
            res.robot_pose.translation() = Eigen::Vector2f(state[2], state[3]);
            res.robot_pose.linear() = Eigen::Rotation2Df(state[4]).toRotationMatrix();
            res.covariance = current_covariance;
            res.timestamp_ms = lidar.second;
            last_update_result = res;
            last_lidar_timestamp = lidar.second;
            qInfo() << "Manual reset: skipping optimization (" << manual_reset_frames_ << " frames remaining)";
            return res;
        }

        // ===== ORIENTATION SEARCH ON FIRST UPDATE =====
        if (needs_orientation_search_)
        {
            const auto state = model_->get_state();
            const float best_phi = find_best_initial_orientation(lidar.first, state[2], state[3], state[4]);
            model_->robot_theta.data().copy_(torch::tensor({best_phi},
                torch::TensorOptions().device(get_device())));
            needs_orientation_search_ = false;
            qInfo() << "Initial orientation search complete. Using phi=" << qRadiansToDegrees(best_phi) << "°";
        }

        // ===== PREDICTION =====
        auto odometry_prior = compute_odometry_prior(velocity_history, lidar);
        const PredictionState prediction = predict_step(model_, odometry_prior, true);

        if (prediction.have_propagated && prediction.propagated_cov.defined())
        {
            auto cov_cpu = prediction.propagated_cov.to(torch::kCPU);
            auto cov_acc = cov_cpu.accessor<float, 2>();
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    current_covariance(i, j) = cov_acc[i][j];
        }

        // ===== DUAL-PRIOR FUSION =====
        auto [pred_pos, pred_theta] = apply_dual_prior_fusion(odometry_prior, odometry_history, lidar);

        // ===== BUILD NEW WINDOW SLOT =====
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
            sampled_points = all_points;

        const torch::Tensor points_tensor = points_to_tensor_xyz(sampled_points, get_device());
        tracking_step_count_++;

        // Motion constraint for the new slot.
        // Prefer measured odometry (encoder/IMU) when available — it is more accurate than
        // the commanded velocity, especially during rotation where wheel slip can differ from
        // the commanded angular rate.  The initial pose (pred_pos, pred_theta) was already
        // computed from the fused estimate inside apply_dual_prior_fusion, so using the same
        // measured delta here removes the command/measured inconsistency that previously caused
        // Adam to fight the motion factor during turns.
        Eigen::Vector3f slot_odom_delta = Eigen::Vector3f::Zero();
        Eigen::Matrix3f slot_motion_cov = Eigen::Matrix3f::Identity() * params.default_slot_motion_cov;
        if (last_measured_prior_.valid)
        {
            slot_odom_delta = last_measured_prior_.delta_pose;
            slot_motion_cov = compute_motion_covariance(last_measured_prior_, true);  // tighter measured noise
        }
        else if (odometry_prior.valid)
        {
            slot_odom_delta = odometry_prior.delta_pose;
            slot_motion_cov = compute_motion_covariance(odometry_prior, false);
        }

        WindowSlot new_slot;
        new_slot.pose = torch::tensor({pred_pos.x(), pred_pos.y(), pred_theta},
            torch::TensorOptions().dtype(torch::kFloat32).device(get_device()).requires_grad(true));
        new_slot.lidar_points = points_tensor;
        new_slot.odometry_delta = slot_odom_delta;
        new_slot.motion_cov = slot_motion_cov;
        new_slot.timestamp_ms = lidar.second;

        // Pre-cache tensors used every Adam iteration.
        // Always build on CPU first (accessor<> requires CPU), then move to device.
        new_slot.odom_delta_tensor = torch::tensor(
            {slot_odom_delta[0], slot_odom_delta[1], slot_odom_delta[2]},
            torch::kFloat32).to(get_device());
        {
            Eigen::Matrix3f prec = slot_motion_cov.inverse();
            auto prec_cpu = torch::zeros({3, 3}, torch::kFloat32);
            auto acc = prec_cpu.accessor<float, 2>();
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    acc[r][c] = prec(r, c);
            new_slot.motion_prec_tensor = prec_cpu.to(get_device());
        }

        const bool window_slid = window_mgr_.append(std::move(new_slot), params.rfe_window_size,
                                                     params.boundary_mu_quality_threshold);
        window_mgr_.subsample_old_slots(params.rfe_max_lidar_per_old_slot);

        // ===== CORNER DETECTION (optional, controlled by EnableCornerTracking) =====
        if (params.enable_corner_tracking && !init_polygon_vertices_.empty())
        {
            auto newest_cpu = window_mgr_.newest().pose.detach().to(torch::kCPU);
            auto pa = newest_cpu.accessor<float, 1>();
            const float cx = pa[0], cy = pa[1], cth = pa[2];

            auto det = corner_detector_.detect(all_points, cx, cy, cth);
            res.corners_in_fov = det.corners_in_fov;
            res.corner_matches = det.matches;

            // Store corner observations in the newest slot for the RFE loss
            auto& newest_slot = window_mgr_.newest();
            newest_slot.corner_obs.clear();
            for (const auto& m : det.matches)
            {
                if (m.model_index < 0 || m.model_index >= static_cast<int>(init_polygon_vertices_.size()))
                    continue;
                WindowSlot::CornerObs obs;
                obs.model_corner_world = init_polygon_vertices_[m.model_index];
                obs.detected_robot = m.detected;
                obs.precision = m.covariance.inverse();
                newest_slot.corner_obs.push_back(obs);
            }
        }

        // ===== EARLY EXIT CHECK =====
        if (auto early = try_prediction_early_exit(points_tensor, slot_odom_delta, odometry_prior, lidar.second))
        {
            // Store quality for future boundary prior gate (early exit = good pose).
            window_mgr_.newest().sdf_mse_final = early->sdf_mse;
            early->corners_in_fov = res.corners_in_fov;
            early->corner_matches = std::move(res.corner_matches);
            early->lidar_scan = std::move(res.lidar_scan);
            return *early;
        }

        // ===== ADAM OPTIMISATION =====
        {
            const auto t0 = std::chrono::high_resolution_clock::now();
            auto [last_loss, iterations] = run_adam_loop(odometry_prior);
            last_t_adam_ms_ = std::chrono::duration<float, std::milli>(
                std::chrono::high_resolution_clock::now() - t0).count();
            res.final_loss     = last_loss;
            res.iterations_used = iterations;
        }

        // ===== FE TERM BREAKDOWN (diagnostic log, no gradient needed) =====
        // Compute only every 5 frames to avoid re-running the full forward pass just for logging.
        // This saves ~4 ms per Adam run (≈5% of spike duration) at the cost of slightly stale
        // per-term breakdown values in the CSV.
        if (params.debug_log_enabled && (tracking_step_count_ % 5 == 0)) {
            const auto t0 = std::chrono::high_resolution_clock::now();
            torch::NoGradGuard no_grad;
            last_loss_breakdown_ = window_mgr_.compute_rfe_loss_breakdown(*model_, params, get_device());
            last_t_breakdown_ms_ = std::chrono::duration<float, std::milli>(
                std::chrono::high_resolution_clock::now() - t0).count();
        } else {
            last_t_breakdown_ms_ = 0.f;
        }

        // ===== COVARIANCE UPDATE =====
        {
            const auto t0 = std::chrono::high_resolution_clock::now();
            auto [covariance, condition_number] = compute_posterior_covariance(points_tensor);
            res.covariance = covariance;
            res.condition_number = condition_number;
            last_t_cov_ms_ = std::chrono::duration<float, std::milli>(
                std::chrono::high_resolution_clock::now() - t0).count();
        }

        // ===== EXTRACT RESULT FROM NEWEST SLOT =====
        res.ok = true;

        {
            auto newest_cpu = window_mgr_.newest().pose.detach().to(torch::kCPU);
            auto p_acc = newest_cpu.accessor<float, 1>();
            float x = p_acc[0], y = p_acc[1], phi = p_acc[2];

            while (phi > M_PI) phi -= 2.0f * M_PI;
            while (phi < -M_PI) phi += 2.0f * M_PI;

            model_->robot_pos.data().copy_(torch::tensor({x, y},
                torch::TensorOptions().device(get_device())));
            model_->robot_theta.data().copy_(torch::tensor({phi},
                torch::TensorOptions().device(get_device())));

            res.sdf_mse = compute_sdf_mse_unscaled(points_tensor, *model_);

            // Store localization quality so future frames can quality-gate the boundary prior
            // (Solutions B & C): when this slot becomes the oldest it carries its own sdf_mse.
            window_mgr_.newest().sdf_mse_final = res.sdf_mse;

            // Build state vector without calling get_state() (avoids 3 GPU→CPU transfers)
            {
                auto ext_cpu = model_->half_extents.to(torch::kCPU);
                auto ext = ext_cpu.accessor<float, 1>();
                res.state << 2.f * ext[0], 2.f * ext[1], x, y, phi;
            }

            Eigen::Affine2f pose = Eigen::Affine2f::Identity();
            pose.translation() = Eigen::Vector2f{x, y};
            pose.linear() = Eigen::Rotation2Df(phi).toRotationMatrix();
            res.robot_pose = pose;

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

        // ===== FINALIZE =====
        if (window_slid && window_mgr_.size() > 0)
            window_mgr_.recompute_boundary_prior(*model_, params, get_device());

        model_->robot_prev_pose = res.robot_pose;

        {
            auto cov_cpu = torch::zeros({3, 3}, torch::kFloat32);
            auto cov_acc = cov_cpu.accessor<float, 2>();
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    cov_acc[i][j] = res.covariance(i, j);
            model_->prev_cov = cov_cpu.to(get_device());
        }

        model_->has_prediction = false;
        res.timestamp_ms = lidar.second;
        last_update_result = res;
        prev_sdf_mse_ = res.sdf_mse;   // track for boundary quality gate next frame

        // ===== DEBUG LOG =====
        if (debug_log_.is_open())
        {
            const auto wall_now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();

            const float vel_adv_y = velocity_history.empty() ? 0.f : velocity_history.back().adv_y;
            const float vel_rot   = velocity_history.empty() ? 0.f : velocity_history.back().rot;
            const float odom_adv  = odometry_history.empty() ? 0.f : odometry_history.back().adv;
            const float odom_rot  = odometry_history.empty() ? 0.f : odometry_history.back().rot;

            const float cmd_cov_xx = last_cmd_cov_(0,0);
            const float cmd_cov_tt = last_cmd_cov_(2,2);
            float meas_cov_xx = 0.f, meas_cov_tt = 0.f;
            if (last_measured_prior_.valid && last_measured_prior_.covariance.defined())
            {
                auto mc = last_measured_prior_.covariance.to(torch::kCPU);
                auto ma = mc.accessor<float, 2>();
                meas_cov_xx = ma[0][0];
                meas_cov_tt = ma[2][2];
            }

            std::string losses_str;
            for (size_t ai = 0; ai < last_adam_losses_.size(); ++ai)
            {
                if (ai > 0) losses_str += '|';
                std::ostringstream ss; ss << last_adam_losses_[ai];
                losses_str += ss.str();
            }
            if (losses_str.empty()) losses_str = "na";

            const float lr_eff = params.learning_rate_pos /
                std::sqrt(static_cast<float>(std::max(1, (int)window_mgr_.size())));

            debug_log_
                << lidar.second
                << ',' << wall_now_ms
                << ',' << odometry_prior.dt
                << ',' << sampled_points.size()
                << ',' << vel_adv_y
                << ',' << vel_rot
                << ',' << odom_adv
                << ',' << odom_rot
                << ',' << (int)odometry_prior.valid
                << ',' << odometry_prior.delta_pose[0]
                << ',' << odometry_prior.delta_pose[1]
                << ',' << odometry_prior.delta_pose[2]
                << ',' << cmd_cov_xx
                << ',' << cmd_cov_tt
                << ',' << (int)last_measured_prior_.valid
                << ',' << last_measured_prior_.delta_pose[0]
                << ',' << last_measured_prior_.delta_pose[1]
                << ',' << last_measured_prior_.delta_pose[2]
                << ',' << meas_cov_xx
                << ',' << meas_cov_tt
                << ',' << pred_pos.x()
                << ',' << pred_pos.y()
                << ',' << pred_theta
                << ',' << slot_motion_cov(0,0)
                << ',' << slot_motion_cov(2,2)
                << ',' << 0                          // early_exit = 0 (Adam ran)
                << ',' << res.iterations_used
                << ',' << res.final_loss
                << ',' << lr_eff
                << ',' << res.state[2]
                << ',' << res.state[3]
                << ',' << res.state[4]
                << ',' << res.innovation[0]
                << ',' << res.innovation[1]
                << ',' << res.innovation[2]
                << ',' << res.innovation_norm
                << ',' << res.sdf_mse
                << ',' << res.covariance(0,0)
                << ',' << res.covariance(2,2)
                << ',' << res.condition_number
                << ',' << (int)window_mgr_.size()
                << ',' << tracking_step_count_
                << ',' << last_loss_breakdown_.boundary
                << ',' << last_loss_breakdown_.obs
                << ',' << last_loss_breakdown_.motion
                << ',' << last_loss_breakdown_.corner
                << ',' << last_loss_init_
                << ',' << std::chrono::duration<float, std::milli>(
                               std::chrono::high_resolution_clock::now() - t_update_start_).count()
                << ',' << last_t_adam_ms_
                << ',' << last_t_cov_ms_
                << ',' << last_t_breakdown_ms_
                << '\n';
            debug_log_.flush();
        }

        return res;
    }

    // =========================================================================
    //  update() helper methods
    // =========================================================================

    std::pair<Eigen::Vector2f, float> RoomConcept::apply_dual_prior_fusion(
        const OdometryPrior& odometry_prior,
        const std::vector<OdometryReading>& odometry_history,
        const std::pair<std::vector<Eigen::Vector3f>, std::int64_t>& lidar)
    {
        auto state = model_->get_state();
        Eigen::Vector2f pred_pos(state[2], state[3]);
        float pred_theta = state[4];

        if (!last_update_result.ok || !odometry_prior.valid)
            return {pred_pos, pred_theta};

        // Command prior: predict from commanded velocity
        const Eigen::Vector2f cmd_pos = last_update_result.robot_pose.translation()
                 + odometry_prior.delta_pose.head<2>();
        float cmd_theta = std::atan2(last_update_result.robot_pose.linear()(1, 0),
                                last_update_result.robot_pose.linear()(0, 0))
                   + odometry_prior.delta_pose[2];
        while (cmd_theta > M_PI) cmd_theta -= 2.0f * M_PI;
        while (cmd_theta < -M_PI) cmd_theta += 2.0f * M_PI;

        const Eigen::Vector3f pred_cmd(cmd_pos.x(), cmd_pos.y(), cmd_theta);

        // Measured odometry prior: predict from encoder/IMU readings
        auto measured_prior = compute_measured_odometry_prior(odometry_history, lidar);
        last_measured_prior_ = measured_prior;  // save for debug log

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

            const Eigen::Matrix3f cov_cmd = compute_motion_covariance(odometry_prior, false);
            last_cmd_cov_ = cov_cmd;  // save for debug log
            const Eigen::Matrix3f cov_odom = compute_motion_covariance(measured_prior, true);

            const auto [fused_mean, fused_precision] = fuse_priors(
                pred_cmd, cov_cmd, pred_odom, cov_odom);

            pred_pos = fused_mean.head<2>();
            pred_theta = fused_mean[2];

            model_->robot_pos.data().copy_(torch::tensor({pred_pos.x(), pred_pos.y()},
                torch::TensorOptions().device(get_device())));
            model_->robot_theta.data().copy_(torch::tensor({pred_theta},
                torch::TensorOptions().device(get_device())));
            model_->set_prediction(pred_pos, pred_theta, fused_precision);
        }
        else
        {
            pred_pos = cmd_pos;
            pred_theta = cmd_theta;

            model_->robot_pos.data().copy_(torch::tensor({pred_pos.x(), pred_pos.y()},
                torch::TensorOptions().device(get_device())));
            model_->robot_theta.data().copy_(torch::tensor({pred_theta},
                torch::TensorOptions().device(get_device())));

            Eigen::Matrix3f prior_precision = current_covariance.inverse();
            model_->set_prediction(pred_pos, pred_theta, prior_precision);
        }

        return {pred_pos, pred_theta};
    }

    std::optional<RoomConcept::UpdateResult> RoomConcept::try_prediction_early_exit(
        const torch::Tensor& points_tensor,
        const Eigen::Vector3f& slot_odom_delta,
        const OdometryPrior& odometry_prior,
        std::int64_t lidar_timestamp_ms)
    {
        // No fast_rotation block here: the SDF quality check below already decides whether
        // the predicted pose (including theta) is accurate enough to skip Adam.
        // If measured odometry is available and accurate, the theta prediction will be good,
        // mean_sdf_pred will be low, and early exit will fire correctly — even during turns.
        // If the prediction is poor (large rotation error), mean_sdf_pred will be high and
        // Adam will run.  An explicit angular-velocity gate was previously needed because
        // slot_odom_delta used command velocity (inconsistent with the fused prediction).
        // Now that slot_odom_delta uses measured odometry, the prediction is self-consistent
        // and the SDF gate alone is sufficient.
        if (!params.prediction_early_exit ||
            !last_update_result.ok ||
            !odometry_prior.valid ||
            tracking_step_count_ <= params.min_tracking_steps)
            return std::nullopt;

        torch::NoGradGuard no_grad;
        const auto& newest = window_mgr_.newest();
        auto pose_xy = newest.pose.index({torch::indexing::Slice(0, 2)});
        auto pose_th = newest.pose.index({torch::indexing::Slice(2, 3)});
        const auto sdf_pred = model_->sdf_at_pose(points_tensor, pose_xy, pose_th);
        const float mean_sdf_pred = torch::mean(torch::abs(sdf_pred)).item<float>();

        // Widen the SDF trust threshold when the robot is rotating.
        // A theta error ε at room scale R produces SDF displacement ~R*ε.
        // The base threshold (sigma_sdf * trust_factor ≈ 7.5 cm) is too tight during rotation:
        // even a 0.02 rad odometry error at 5 m gives ~10 cm — larger than the base threshold.
        // We add rotation_sdf_coupling * |delta_theta| to compensate for this geometric effect.
        const float rot_boost = params.rotation_sdf_coupling * std::abs(odometry_prior.delta_pose[2]);
        const float prediction_trust_threshold = params.sigma_sdf * params.prediction_trust_factor + rot_boost;
        if (mean_sdf_pred >= prediction_trust_threshold)
            return std::nullopt;

        prediction_early_exits_++;

        auto pose_cpu = newest.pose.detach().to(torch::kCPU);
        auto p_acc = pose_cpu.accessor<float, 1>();
        const float x = p_acc[0], y = p_acc[1], phi = p_acc[2];

        UpdateResult res;
        res.ok = true;
        res.final_loss = mean_sdf_pred;
        res.sdf_mse = mean_sdf_pred;
        res.iterations_used = 0;
        {
            auto ext_cpu = model_->half_extents.to(torch::kCPU);
            auto ext = ext_cpu.accessor<float, 1>();
            res.state << 2.f * ext[0], 2.f * ext[1], x, y, phi;
        }

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

        model_->robot_pos.data().copy_(torch::tensor({x, y},
            torch::TensorOptions().device(get_device())));
        model_->robot_theta.data().copy_(torch::tensor({phi},
            torch::TensorOptions().device(get_device())));
        model_->robot_prev_pose = res.robot_pose;
        model_->has_prediction = false;

        res.timestamp_ms = lidar_timestamp_ms;
        last_update_result = res;
        prev_sdf_mse_ = res.sdf_mse;   // track for boundary quality gate next frame
        last_lidar_timestamp = lidar_timestamp_ms;

        // Debug log for early-exit frames
        if (debug_log_.is_open())
        {
            const auto wall_now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            const float lr_eff = params.learning_rate_pos /
                std::sqrt(static_cast<float>(std::max(1, (int)window_mgr_.size())));
            float meas_cov_xx = 0.f, meas_cov_tt = 0.f;
            if (last_measured_prior_.valid && last_measured_prior_.covariance.defined())
            {
                auto mc = last_measured_prior_.covariance.to(torch::kCPU);
                auto ma = mc.accessor<float, 2>();
                meas_cov_xx = ma[0][0]; meas_cov_tt = ma[2][2];
            }
            debug_log_
                << lidar_timestamp_ms
                << ',' << wall_now_ms
                << ',' << odometry_prior.dt
                << ',' << 0           // n_lidar not available here
                << ',' << 0 << ',' << 0 << ',' << 0 << ',' << 0  // vel/odom
                << ',' << (int)odometry_prior.valid
                << ',' << odometry_prior.delta_pose[0]
                << ',' << odometry_prior.delta_pose[1]
                << ',' << odometry_prior.delta_pose[2]
                << ',' << last_cmd_cov_(0,0) << ',' << last_cmd_cov_(2,2)
                << ',' << (int)last_measured_prior_.valid
                << ',' << last_measured_prior_.delta_pose[0]
                << ',' << last_measured_prior_.delta_pose[1]
                << ',' << last_measured_prior_.delta_pose[2]
                << ',' << meas_cov_xx << ',' << meas_cov_tt
                << ',' << x << ',' << y << ',' << phi
                << ',' << 0 << ',' << 0   // slot_mcov (not available here)
                << ',' << 1               // early_exit = 1
                << ',' << 0               // iters
                << ',' << mean_sdf_pred
                << ',' << lr_eff
                << ',' << x << ',' << y << ',' << phi
                << ',' << res.innovation[0] << ',' << res.innovation[1] << ',' << res.innovation[2]
                << ',' << res.innovation_norm
                << ',' << res.sdf_mse
                << ',' << res.covariance(0,0) << ',' << res.covariance(2,2)
                << ',' << 0               // cond_num
                << ',' << (int)window_mgr_.size()
                << ',' << tracking_step_count_
                << ',' << "nan" << ',' << "nan" << ',' << "nan" << ',' << "nan"  // loss_boundary/obs/motion/corner
                << ',' << "nan"  // loss_init
                << ',' << std::chrono::duration<float, std::milli>(
                               std::chrono::high_resolution_clock::now() - t_update_start_).count()
                << ',' << 0.f << ',' << 0.f << ',' << 0.f  // t_adam, t_cov, t_breakdown
                << '\n';
            debug_log_.flush();
        }

        return res;
    }

    std::pair<float, int> RoomConcept::run_adam_loop(const OdometryPrior& odometry_prior)
    {
        auto window_params = window_mgr_.collect_params();

        const int ws = static_cast<int>(window_mgr_.size());
        const float ws_scale = 1.0f / std::sqrt(static_cast<float>(std::max(1, ws)));
        const float lr = params.learning_rate_pos * ws_scale;
        torch::optim::Adam optimizer(
            {torch::optim::OptimizerParamGroup(window_params,
                std::make_unique<torch::optim::AdamOptions>(lr))});

        const Eigen::Vector3f velocity_weights = compute_velocity_adaptive_weights(odometry_prior);

        // ===== Boundary quality gate =====
        // Scale the boundary prior by how trustworthy the previous frame's pose was.
        // w = min(1, sigma_sdf² / sdf_mse_prev)
        // Good prev pose (sdf_mse_prev ≈ 0) → w≈1 (strong prior, normal behaviour).
        // Bad  prev pose (sdf_mse_prev >> sigma_sdf) → w→0 (prior suppressed, ADAM free to recover).
        float boundary_weight = 1.0f;
        if (params.rfe_boundary_quality_gate && prev_sdf_mse_ > 1e-6f)
        {
            const float sigma2 = params.sigma_sdf * params.sigma_sdf;
            boundary_weight = std::min(1.0f, sigma2 / prev_sdf_mse_);
        }

        float last_loss = std::numeric_limits<float>::infinity();
        float prev_loss = std::numeric_limits<float>::infinity();
        int iterations = 0;

        last_adam_losses_.clear();
        last_adam_losses_.reserve(params.num_iterations);
        last_loss_init_ = 0.f;

        for (int i = 0; i < params.num_iterations; ++i)
        {
            optimizer.zero_grad();

            const torch::Tensor loss = window_mgr_.compute_rfe_loss(*model_, params, get_device(),
                                                                      boundary_weight);

            // Record initial loss (before any parameter update) for convergence diagnostics
            if (i == 0)
                last_loss_init_ = loss.item<float>();

            loss.backward();

            {
                torch::NoGradGuard no_grad;
                auto& newest_pose = window_mgr_.newest().pose;
                if (newest_pose.grad().defined())
                {
                    newest_pose.mutable_grad().index({0}) *= velocity_weights[0];
                    newest_pose.mutable_grad().index({1}) *= velocity_weights[1];
                    newest_pose.mutable_grad().index({2}) *= velocity_weights[2];
                }
            }

            optimizer.step();

            prev_loss = last_loss;
            last_loss = loss.item<float>();
            last_adam_losses_.push_back(last_loss);
            iterations = i + 1;

            if (last_loss < params.min_loss_threshold)
                break;
            if (i > params.convergence_min_iters &&
                std::abs(prev_loss - last_loss) < params.convergence_relative_tol * prev_loss)
                break;
        }

        return {last_loss, iterations};
    }

    std::pair<Eigen::Matrix3f, float> RoomConcept::compute_posterior_covariance(
        const torch::Tensor& points_tensor)
    {
        try {
            auto& newest = window_mgr_.newest();
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

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(H_likelihood);
            Eigen::Vector3f evals = eig.eigenvalues().cwiseMax(params.eigenvalue_clamp_posterior);
            H_likelihood = eig.eigenvectors() * evals.asDiagonal() * eig.eigenvectors().transpose();

            Eigen::Matrix3f prior_precision = current_covariance.inverse();
            const float lambda = params.covariance_regularization;
            Eigen::Matrix3f posterior_precision = prior_precision + H_likelihood
                                                 + lambda * Eigen::Matrix3f::Identity();

            Eigen::Matrix3f new_cov = posterior_precision.inverse();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(posterior_precision);
            const auto eigenvalues = solver.eigenvalues();
            const float max_ev = eigenvalues.maxCoeff();
            const float min_ev = eigenvalues.minCoeff();
            const float cond = (min_ev > 1e-8f) ? (max_ev / min_ev) : 1e8f;

            if (new_cov.allFinite() && new_cov.determinant() > params.covariance_det_min
                && cond < params.condition_number_max)
            {
                current_covariance = new_cov;
                return {new_cov, cond};
            }
            return {current_covariance, cond};
        } catch (const std::exception &e) {
            std::cerr << "RFE covariance update failed: " << e.what() << std::endl;
            return {current_covariance, -1.0f};
        }
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
        const float noise_trans = is_measured_odometry ? params.odom_noise_trans * params.odom_noise_scale : params.cmd_noise_trans;
        const float noise_rot   = is_measured_odometry ? params.odom_noise_rot   * params.odom_noise_scale : params.cmd_noise_rot;
        const float noise_base  = is_measured_odometry ? params.odom_noise_base  * params.odom_noise_scale : params.cmd_noise_base;

        float motion_magnitude = std::sqrt(
            odometry_prior.delta_pose[0] * odometry_prior.delta_pose[0] +
            odometry_prior.delta_pose[1] * odometry_prior.delta_pose[1]
        );

        // Uncertainty grows with distance; when stationary use tight constraint
        float base_uncertainty;
        if (motion_magnitude < params.stationary_motion_threshold) {
            base_uncertainty = params.stationary_motion_threshold;
        } else {
            base_uncertainty = noise_base;
        }

        float position_std = base_uncertainty + noise_trans * motion_magnitude;
        float rotation_std = params.rotation_noise_base + noise_rot * std::abs(odometry_prior.delta_pose[2]);

        // Rotation-position coupling: rotation creates position uncertainty
        // (pivot wobble, wheel slip, lever arm effects)
        float rot_induced_pos = params.rotation_position_coupling * std::abs(odometry_prior.delta_pose[2]);
        position_std = std::sqrt(position_std * position_std + rot_induced_pos * rot_induced_pos);

        // Encoder angular slip model (measured odometry only):
        // At high angular speeds, wheel encoders under-report rotation due to slip.
        // Inflate rotation variance proportionally to angular speed so the Bayesian
        // fusion reduces the encoder's weight relative to the command predictor.
        if (is_measured_odometry && params.encoder_rot_slip_k > 0.f)
        {
            const float ang_speed = std::abs(odometry_prior.delta_pose[2]) /
                                    std::max(odometry_prior.dt * 0.001f, 0.001f);
            const float slip_std  = params.encoder_rot_slip_k * ang_speed;
            rotation_std = std::sqrt(rotation_std * rotation_std + slip_std * slip_std);
        }

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
        Eigen::Vector3f total_delta = Eigen::Vector3f::Zero();

        float running_theta = std::atan2(robot_pose.linear()(1,0), robot_pose.linear()(0,0));

        // Integrate over all velocity commands in [t_start_ms, t_end_ms] using source/recv epoch-ms.
        for (size_t i = 0; i < velocity_history.size(); ++i) {
            const auto& vcmd = velocity_history[i];

            // Get time window for this command
            const std::int64_t cmd_start_ms = vcmd.effective_ts_ms();
            const std::int64_t cmd_end_ms = (i + 1 < velocity_history.size())
                           ? velocity_history[i + 1].effective_ts_ms()
                           : t_end_ms;

            if (cmd_start_ms <= 0 || cmd_end_ms <= 0 || cmd_end_ms <= cmd_start_ms)
                continue;

            // Clip to [t_start_ms, t_end_ms]
            if (cmd_end_ms < t_start_ms) continue;
            if (cmd_start_ms > t_end_ms) break;

            const std::int64_t effective_start_ms = std::max(cmd_start_ms, t_start_ms);
            const std::int64_t effective_end_ms = std::min(cmd_end_ms, t_end_ms);

            const float dt = static_cast<float>(effective_end_ms - effective_start_ms) * 0.001f;
            if (dt <= 0) continue;

            // Integrate this segment
            const float dx_local = (vcmd.adv_x * dt);
            const float dy_local = (vcmd.adv_y * dt);

            const float dtheta = vcmd.rot * dt;   // velocity buffer is CCW+; use directly

            // Transform to global frame using MIDPOINT theta (reduces integration bias)
            const float theta_mid = running_theta + 0.5f * dtheta;
            total_delta[0] += dx_local * std::cos(theta_mid) - dy_local * std::sin(theta_mid);
            total_delta[1] += dx_local * std::sin(theta_mid) + dy_local * std::cos(theta_mid);
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

        // Reuse pre-allocated F (identity + off-diag) and Q (zeroed + filled)
        if (predict_alloc_dim_ != dim)
        {
            predict_F_ = torch::eye(dim, torch::TensorOptions().dtype(torch::kFloat32).device(device));
            predict_Q_ = torch::zeros({dim, dim}, torch::TensorOptions().dtype(torch::kFloat32).device(device));
            predict_alloc_dim_ = dim;
        }
        // Reset F off-diagonal and Q to zero for this frame
        auto F = predict_F_;
        auto Q = predict_Q_;
        // Reset the variable elements
        if (is_localized) {
            // ∂x/∂θ = -dx_local·sin(θ) - dy_local·cos(θ)
            // ∂y/∂θ =  dx_local·cos(θ) - dy_local·sin(θ)
            F[0][2] = -dx_local * sin_t - dy_local * cos_t;
            F[1][2] =  dx_local * cos_t - dy_local * sin_t;
        } else {
            F[2][4] = -dx_local * sin_t - dy_local * cos_t;
            F[3][4] =  dx_local * cos_t - dy_local * sin_t;
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
        const bool near_stationary = linear_speed < params.stationary_speed_threshold && angular_speed < params.stationary_speed_threshold;

        // Base noise grows with elapsed time (random-walk style) and is heavily reduced at rest.
        // This keeps slow covariance inflation when stopped, avoiding constant re-minimization.
        const float time_scale = std::sqrt(std::min(1.0f, 4.0f * dt_s));
        float base_trans_noise = params.cmd_noise_base * time_scale;
        if (near_stationary)
            base_trans_noise *= params.stationary_noise_damping;

        // Forward uncertainty: grows with forward motion
        float forward_noise = base_trans_noise + params.cmd_noise_trans * forward_motion;

        // Lateral uncertainty: smaller for differential drive
        float lateral_noise = base_trans_noise + params.lateral_noise_fraction * params.cmd_noise_trans * lateral_motion;

        // Rotation noise: base + motion-dependent
        float base_rot_noise = params.base_rotation_noise_fraction * base_trans_noise;
        float rot_noise = base_rot_noise + params.cmd_noise_rot * std::abs(dtheta);

        Q.zero_();

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
        Eigen::Vector3f total_delta = Eigen::Vector3f::Zero();
        float running_theta = std::atan2(robot_pose.linear()(1,0), robot_pose.linear()(0,0));

        // Integrate over all odometry readings in [t_start_ms, t_end_ms] using source/recv epoch-ms.
        for (size_t i = 0; i < odometry_history.size(); ++i)
        {
            const auto& odom = odometry_history[i];

            // Get time window for this reading
            const std::int64_t cmd_start_ms = odom.effective_ts_ms();
            const std::int64_t cmd_end_ms = (i + 1 < odometry_history.size())
                           ? odometry_history[i + 1].effective_ts_ms()
                           : t_end_ms;

            if (cmd_start_ms <= 0 || cmd_end_ms <= 0 || cmd_end_ms <= cmd_start_ms)
                continue;

            // Clip to [t_start_ms, t_end_ms]
            if (cmd_end_ms < t_start_ms) continue;
            if (cmd_start_ms > t_end_ms) break;

            const std::int64_t effective_start_ms = std::max(cmd_start_ms, t_start_ms);
            const std::int64_t effective_end_ms = std::min(cmd_end_ms, t_end_ms);

            const float dt = static_cast<float>(effective_end_ms - effective_start_ms) * 0.001f;
            if (dt <= 0) continue;

            // Odometry velocities are in robot frame: adv=forward(Y), side=lateral(X), rot=angular
            const float dx_local = odom.side * dt;   // lateral (X in robot frame)
            const float dy_local = odom.adv * dt;    // forward (Y in robot frame)
            const float dtheta = odom.rot * dt;      // odometry buffer is CCW+; use directly

            // Transform to global frame using MIDPOINT theta (reduces integration bias)
            const float theta_mid = running_theta + 0.5f * dtheta;
            total_delta[0] += dx_local * std::cos(theta_mid) - dy_local * std::sin(theta_mid);
            total_delta[1] += dx_local * std::sin(theta_mid) + dy_local * std::cos(theta_mid);
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

        const int64_t prev_ts = last_update_result.timestamp_ms;
        if (prev_ts == 0 || odometry_history.empty() || !last_update_result.ok)
            return prior;
        const auto dt = lidar_timestamp - prev_ts;
        // prior.delta_pose = integrate_odometry_over_window(..., prev_ts, lidar_timestamp);

        // if (last_lidar_timestamp == 0 || odometry_history.empty() || !last_update_result.ok)
        //     return prior;
        // const auto dt = lidar_timestamp - last_lidar_timestamp;
        if (dt <= 0)
            return prior;
        prior.dt = static_cast<float>(dt);

        prior.delta_pose = integrate_odometry_over_window(
            last_update_result.robot_pose,
            odometry_history,
            prev_ts,
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
    //  Differential test: shadow single-step Adam evaluator
    // =====================================================================
    float RoomConcept::shadow_single_step_adam(const torch::Tensor& points_tensor,
                                               float pred_x, float pred_y, float pred_theta) const
    {
        const auto device = get_device();

        // Create an isolated pose tensor starting from the predicted pose
        auto pose = torch::tensor({pred_x, pred_y, pred_theta},
            torch::TensorOptions().dtype(torch::kFloat32).device(device)).requires_grad_(true);

        const float inv_var = 1.0f / (params.rfe_obs_sigma * params.rfe_obs_sigma);
        const float huber_delta = params.rfe_huber_delta;

        torch::optim::Adam optimizer(
            {torch::optim::OptimizerParamGroup({pose},
                std::make_unique<torch::optim::AdamOptions>(params.learning_rate_pos))});

        // Run the same number of iterations as the main loop
        for (int i = 0; i < params.num_iterations; ++i)
        {
            optimizer.zero_grad();
            auto xy = pose.index({torch::indexing::Slice(0, 2)});
            auto th = pose.index({torch::indexing::Slice(2, 3)});
            auto sdf_vals = model_->sdf_at_pose(points_tensor, xy, th);
            auto loss = 0.5f * inv_var * torch::nn::functional::huber_loss(
                sdf_vals, torch::zeros_like(sdf_vals),
                torch::nn::functional::HuberLossFuncOptions().reduction(torch::kMean).delta(huber_delta));
            loss.backward();
            optimizer.step();
        }

        // Evaluate SDF at optimised pose (same metric as main pipeline)
        torch::NoGradGuard no_grad;
        auto xy = pose.index({torch::indexing::Slice(0, 2)});
        auto th = pose.index({torch::indexing::Slice(2, 3)});
        auto sdf_vals = model_->sdf_at_pose(points_tensor, xy, th);
        return torch::median(torch::abs(sdf_vals)).item<float>();
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
    //  WindowManager methods
    // =====================================================================

    bool RoomConcept::WindowManager::append(WindowSlot slot, int max_window_size,
                                             float mu_quality_threshold)
    {
        bool slid = false;
        if (static_cast<int>(window.size()) >= max_window_size)
        {
            auto pose_cpu = window.front().pose.detach().to(torch::kCPU);
            auto pose_acc = pose_cpu.accessor<float, 1>();

            // Solution C: only update boundary mu if the dropped slot had acceptable
            // localization quality. If the slot was confused (displacement, obstacle),
            // keep the previous mu so the prior continues anchoring to the last good pose.
            const bool slot_is_good = (window.front().sdf_mse_final < mu_quality_threshold)
                                      || !boundary_prior.valid;
            if (slot_is_good)
                boundary_prior.mu = Eigen::Vector3f(pose_acc[0], pose_acc[1], pose_acc[2]);

            window.pop_front();
            slid = true;
        }
        window.push_back(std::move(slot));
        return slid;
    }

    void RoomConcept::WindowManager::subsample_old_slots(int max_pts_per_slot)
    {
        if (window.size() <= 1 || max_pts_per_slot <= 0) return;
        for (size_t i = 0; i < window.size() - 1; i++)
        {
            auto& slot = window[i];
            if (slot.subsampled) continue;   // already done — skip
            const int64_t n_pts = slot.lidar_points.size(0);
            if (n_pts > max_pts_per_slot)
            {
                const int64_t stride = n_pts / max_pts_per_slot;
                auto indices = torch::arange(0, n_pts, stride,
                    torch::TensorOptions().dtype(torch::kLong).device(slot.lidar_points.device()));
                slot.lidar_points = slot.lidar_points.index_select(0, indices).contiguous();
            }
            slot.subsampled = true;
        }
    }

    std::vector<torch::Tensor> RoomConcept::WindowManager::collect_params() const
    {
        std::vector<torch::Tensor> p;
        p.reserve(window.size());
        for (const auto& slot : window)
            p.push_back(slot.pose);
        return p;
    }

    torch::Tensor RoomConcept::WindowManager::compute_rfe_loss(
        const Model& model, const Params& params, torch::Device device,
        float boundary_weight) const
    {
        if (window.empty())
            return torch::tensor(0.0f, torch::TensorOptions().device(device));

        const float inv_var = 1.0f / (params.rfe_obs_sigma * params.rfe_obs_sigma);
        const float huber_delta = params.rfe_huber_delta;

        torch::Tensor total_loss = torch::tensor(0.0f, torch::TensorOptions().device(device));

        // --- 1. Boundary prior on oldest surviving state (Eq. 28) ---
        // With W=1 the oldest slot IS the current slot: the prior would anchor the current
        // pose to the previous frame's post-ADAM estimate, creating an error integrator that
        // drives sawtooth drift even when the robot is static.  Only apply when W > 1.
        if (boundary_prior.valid && window.size() > 1)
        {
            const auto& oldest_pose = window.front().pose;
            const auto mu = torch::tensor(
                {boundary_prior.mu[0], boundary_prior.mu[1], boundary_prior.mu[2]},
                torch::TensorOptions().dtype(torch::kFloat32).device(device));

            auto prec_data = torch::zeros({3, 3}, torch::TensorOptions().dtype(torch::kFloat32).device(device));
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    prec_data[i][j] = boundary_prior.precision(i, j);

            auto raw_diff = oldest_pose - mu;
            auto angle_diff = raw_diff.index({2});
            auto wrapped_diff = torch::cat({raw_diff.index({torch::indexing::Slice(0, 2)}),
                                             torch::atan2(torch::sin(angle_diff), torch::cos(angle_diff)).unsqueeze(0)});
            auto diff = wrapped_diff.unsqueeze(1);
            auto boundary_loss = boundary_weight *
                0.5f * torch::matmul(diff.t(), torch::matmul(prec_data, diff)).squeeze();
            total_loss = total_loss + boundary_loss;
        }

        // --- 2. Observation factors: SDF likelihood at each timestep ---
        for (const auto& slot : window)
        {
            auto pose_xy = slot.pose.index({torch::indexing::Slice(0, 2)});
            auto pose_theta = slot.pose.index({torch::indexing::Slice(2, 3)});

            const auto sdf_vals = model.sdf_at_pose(slot.lidar_points, pose_xy, pose_theta);

            torch::Tensor slot_obs_loss;
            if (params.far_points_weight && slot.lidar_points.size(0) > 1)
            {
                // Distance-proportional weighting with configurable exponent α.
                // w_i = dist_i^α / mean(dist^α)  →  mean(w) ≈ 1  →  no loss-scale shift.
                // After clamping to [far_points_min_weight, ∞) we re-normalise so the
                // mean stays at 1 even when many near points hit the floor.
                auto pts_xy = slot.lidar_points.index(
                    {torch::indexing::Slice(), torch::indexing::Slice(0, 2)});
                auto dists       = torch::norm(pts_xy, 2, /*dim=*/1);                    // [N]
                auto dists_alpha = torch::pow(dists, params.far_points_exponent);        // [N]
                auto weights     = dists_alpha / (dists_alpha.mean() + 1e-6f);           // [N], mean≈1
                weights          = weights.clamp_min(params.far_points_min_weight);      // floor
                weights          = weights / (weights.mean() + 1e-6f);                   // re-normalise

                auto per_point = torch::nn::functional::huber_loss(
                    sdf_vals,
                    torch::zeros_like(sdf_vals),
                    torch::nn::functional::HuberLossFuncOptions()
                        .reduction(torch::kNone).delta(huber_delta));                    // [N]

                slot_obs_loss = 0.5f * inv_var * (per_point * weights).mean();
            }
            else
            {
                const auto obs_huber = torch::nn::functional::huber_loss(
                    sdf_vals,
                    torch::zeros({sdf_vals.size(0)}, sdf_vals.options()),
                    torch::nn::functional::HuberLossFuncOptions()
                        .reduction(torch::kMean).delta(huber_delta));
                slot_obs_loss = 0.5f * inv_var * obs_huber;
            }
            total_loss = total_loss + slot_obs_loss;
        }

        // --- 3. Motion factors between consecutive slots ---
        for (size_t i = 1; i < window.size(); i++)
        {
            const auto& curr = window[i];
            const auto& prev = window[i - 1];

            auto pose_delta = curr.pose - prev.pose;
            auto raw_residual = pose_delta - curr.odom_delta_tensor;

            auto angle_res = raw_residual.index({2});
            auto wrapped_angle = torch::atan2(torch::sin(angle_res), torch::cos(angle_res));
            auto residual = torch::cat({raw_residual.index({torch::indexing::Slice(0, 2)}),
                                        wrapped_angle.unsqueeze(0)});

            auto res_col = residual.unsqueeze(1);
            auto motion_loss = 0.5f * torch::matmul(res_col.t(), torch::matmul(curr.motion_prec_tensor, res_col)).squeeze();
            total_loss = total_loss + motion_loss;
        }

        // --- 4. Corner observation factors (newest slots only, Huber-saturated) ---
        if (params.enable_corner_tracking)
        {
        const float corner_inv_var = 1.0f / (params.corner_obs_sigma * params.corner_obs_sigma);
        const float corner_huber = params.corner_huber_delta;
        const int corner_start = std::max(0, static_cast<int>(window.size()) - params.corner_max_slots);
        for (size_t si = corner_start; si < window.size(); si++)
        {
            const auto& slot = window[si];
            if (slot.corner_obs.empty()) continue;

            auto pose_xy    = slot.pose.index({torch::indexing::Slice(0, 2)});
            auto pose_theta = slot.pose.index({2});
            auto cos_th = torch::cos(pose_theta);
            auto sin_th = torch::sin(pose_theta);

            for (const auto& obs : slot.corner_obs)
            {
                // Predicted observation: z_hat = R(-θ) · (c_world - t)
                auto c_w = torch::tensor({obs.model_corner_world.x(), obs.model_corner_world.y()},
                    torch::TensorOptions().dtype(torch::kFloat32).device(device));
                auto dw = c_w - pose_xy;
                auto pred_x = cos_th * dw.index({0}) + sin_th * dw.index({1});
                auto pred_y = -sin_th * dw.index({0}) + cos_th * dw.index({1});
                auto predicted = torch::stack({pred_x, pred_y});

                auto detected = torch::tensor({obs.detected_robot.x(), obs.detected_robot.y()},
                    torch::TensorOptions().dtype(torch::kFloat32).device(device));

                auto residual = detected - predicted;

                // Huber-saturated Mahalanobis: use only per-corner precision (no double σ²)
                auto r_sq = torch::dot(residual, residual);
                auto r_norm = torch::sqrt(r_sq + 1e-8f);
                auto huber_weight = torch::where(r_norm <= corner_huber,
                    torch::ones_like(r_norm),
                    corner_huber / r_norm);
                auto corner_loss = 0.5f * corner_inv_var * huber_weight * r_sq;
                total_loss = total_loss + corner_loss;
            }
        }
        } // enable_corner_tracking

        return total_loss;
    }

    RoomConcept::WindowManager::LossBreakdown
    RoomConcept::WindowManager::compute_rfe_loss_breakdown(
        const Model& model, const Params& params, torch::Device device) const
    {
        LossBreakdown bd;
        if (window.empty()) return bd;

        const float inv_var      = 1.0f / (params.rfe_obs_sigma * params.rfe_obs_sigma);
        const float huber_delta  = params.rfe_huber_delta;

        // 1. Boundary prior
        if (boundary_prior.valid) {
            const auto& oldest_pose = window.front().pose;
            auto mu = torch::tensor(
                {boundary_prior.mu[0], boundary_prior.mu[1], boundary_prior.mu[2]},
                torch::TensorOptions().dtype(torch::kFloat32).device(device));
            auto prec_data = torch::zeros({3, 3}, torch::TensorOptions().dtype(torch::kFloat32).device(device));
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    prec_data[i][j] = boundary_prior.precision(i, j);
            auto raw_diff = oldest_pose.detach() - mu;
            auto angle_diff = raw_diff.index({2});
            auto wrapped_diff = torch::cat({raw_diff.index({torch::indexing::Slice(0, 2)}),
                torch::atan2(torch::sin(angle_diff), torch::cos(angle_diff)).unsqueeze(0)});
            auto diff = wrapped_diff.unsqueeze(1);
            bd.boundary = (0.5f * torch::matmul(diff.t(), torch::matmul(prec_data, diff)).squeeze()).item<float>();
        }

        // 2. Observation factors
        float obs_acc = 0.f;
        for (const auto& slot : window) {
            auto pose_xy    = slot.pose.detach().index({torch::indexing::Slice(0, 2)});
            auto pose_theta = slot.pose.detach().index({torch::indexing::Slice(2, 3)});
            const auto sdf_vals = model.sdf_at_pose(slot.lidar_points, pose_xy, pose_theta);
            float slot_loss;
            if (params.far_points_weight && slot.lidar_points.size(0) > 1) {
                auto pts_xy      = slot.lidar_points.index(
                    {torch::indexing::Slice(), torch::indexing::Slice(0, 2)});
                auto dists       = torch::norm(pts_xy, 2, 1);
                auto dists_alpha = torch::pow(dists, params.far_points_exponent);
                auto weights     = dists_alpha / (dists_alpha.mean() + 1e-6f);
                weights          = weights.clamp_min(params.far_points_min_weight);
                weights          = weights / (weights.mean() + 1e-6f);
                auto per_point = torch::nn::functional::huber_loss(
                    sdf_vals, torch::zeros_like(sdf_vals),
                    torch::nn::functional::HuberLossFuncOptions()
                        .reduction(torch::kNone).delta(huber_delta));
                slot_loss = (0.5f * inv_var * (per_point * weights).mean()).item<float>();
            } else {
                slot_loss = (0.5f * inv_var *
                    torch::nn::functional::huber_loss(
                        sdf_vals,
                        torch::zeros({sdf_vals.size(0)}, sdf_vals.options()),
                        torch::nn::functional::HuberLossFuncOptions()
                            .reduction(torch::kMean).delta(huber_delta)
                    )).item<float>();
            }
            obs_acc += slot_loss;
        }
        bd.obs = obs_acc;

        // 3. Motion factors
        float mot_acc = 0.f;
        for (size_t i = 1; i < window.size(); i++) {
            const auto& curr = window[i];
            const auto& prev = window[i - 1];
            auto pose_delta  = curr.pose.detach() - prev.pose.detach();
            auto raw_residual = pose_delta - curr.odom_delta_tensor;
            auto angle_res   = raw_residual.index({2});
            auto wrapped_angle = torch::atan2(torch::sin(angle_res), torch::cos(angle_res));
            auto residual = torch::cat({raw_residual.index({torch::indexing::Slice(0, 2)}),
                                        wrapped_angle.unsqueeze(0)});
            auto res_col = residual.unsqueeze(1);
            mot_acc += (0.5f * torch::matmul(res_col.t(),
                torch::matmul(curr.motion_prec_tensor, res_col)).squeeze()).item<float>();
        }
        bd.motion = mot_acc;

        // 4. Corner factors
        if (params.enable_corner_tracking) {
            const float corner_inv_var = 1.0f / (params.corner_obs_sigma * params.corner_obs_sigma);
            const float corner_huber   = params.corner_huber_delta;
            const int corner_start = std::max(0, static_cast<int>(window.size()) - params.corner_max_slots);
            float cor_acc = 0.f;
            for (size_t si = corner_start; si < window.size(); si++) {
                const auto& slot = window[si];
                if (slot.corner_obs.empty()) continue;
                auto pose_xy    = slot.pose.detach().index({torch::indexing::Slice(0, 2)});
                auto pose_theta = slot.pose.detach().index({2});
                auto cos_th = torch::cos(pose_theta);
                auto sin_th = torch::sin(pose_theta);
                for (const auto& obs : slot.corner_obs) {
                    auto c_w = torch::tensor({obs.model_corner_world.x(), obs.model_corner_world.y()},
                        torch::TensorOptions().dtype(torch::kFloat32).device(device));
                    auto dw = c_w - pose_xy;
                    auto pred_x = cos_th * dw.index({0}) + sin_th * dw.index({1});
                    auto pred_y = -sin_th * dw.index({0}) + cos_th * dw.index({1});
                    auto predicted = torch::stack({pred_x, pred_y});
                    auto detected  = torch::tensor({obs.detected_robot.x(), obs.detected_robot.y()},
                        torch::TensorOptions().dtype(torch::kFloat32).device(device));
                    auto residual  = detected - predicted;
                    auto r_sq  = torch::dot(residual, residual);
                    auto r_norm = torch::sqrt(r_sq + 1e-8f);
                    auto hw = torch::where(r_norm <= corner_huber, torch::ones_like(r_norm), corner_huber / r_norm);
                    cor_acc += (0.5f * corner_inv_var * hw * r_sq).item<float>();
                }
            }
            bd.corner = cor_acc;
        }

        return bd;
    }

    void RoomConcept::WindowManager::recompute_boundary_prior(
        const Model& model, const Params& params, torch::Device device)
    {
        if (window.empty())
            return;

        const auto& oldest = window.front();

        // ── mu update (Solution C already applied in append()) ────────────────
        // By this point boundary_prior.mu was already conditionally updated when the
        // slot was dropped from the window. We only need to update it here for the
        // "recompute after recovery / reset" path where append() was not called.
        // Unconditional write is safe: recompute is only called when window_slid==true
        // and the slot that slid out already had its quality checked in append().
        // (No additional guard needed here — the pose in mu was set correctly.)

        // ── Hessian computation (Solution B) ─────────────────────────────────
        // If the oldest slot's scan was of poor quality (sdf_mse_final exceeds the
        // threshold), the H_obs term would encode a high-confidence direction toward
        // a contaminated pose. In that case we use only the kinematic (motion) factor
        // for the precision matrix, which is always trustworthy.
        const bool use_obs_hessian =
            (oldest.sdf_mse_final < params.boundary_hessian_quality_threshold);

        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();

        if (use_obs_hessian)
        {
            // ── Full path: H_obs + H_motion + H_prior (original behaviour) ───
            auto oldest_pose_for_hess = oldest.pose.clone().detach().requires_grad_(true);
            auto pose_xy    = oldest_pose_for_hess.index({torch::indexing::Slice(0, 2)});
            auto pose_theta = oldest_pose_for_hess.index({torch::indexing::Slice(2, 3)});

            const float inv_var = 1.0f / (params.rfe_obs_sigma * params.rfe_obs_sigma);
            auto sdf_vals = model.sdf_at_pose(oldest.lidar_points, pose_xy, pose_theta);
            auto loss = 0.5f * inv_var * torch::nn::functional::huber_loss(
                sdf_vals, torch::zeros({sdf_vals.size(0)}, sdf_vals.options()),
                torch::nn::functional::HuberLossFuncOptions().reduction(torch::kMean).delta(params.rfe_huber_delta));

            if (window.size() > 1)
            {
                const auto& next = window[1];
                auto next_pose = next.pose.detach();
                auto delta = next_pose - oldest_pose_for_hess;
                auto raw_res = delta - next.odom_delta_tensor;
                auto angle_r = raw_res.index({2});
                auto residual = torch::cat({raw_res.index({torch::indexing::Slice(0, 2)}),
                                            torch::atan2(torch::sin(angle_r), torch::cos(angle_r)).unsqueeze(0)});
                auto res_col = residual.unsqueeze(1);
                loss = loss + 0.5f * torch::matmul(res_col.t(),
                                                    torch::matmul(next.motion_prec_tensor, res_col)).squeeze();
            }

            if (boundary_prior.valid)
            {
                auto mu_t = torch::tensor(
                    {boundary_prior.mu[0], boundary_prior.mu[1], boundary_prior.mu[2]},
                    torch::TensorOptions().dtype(torch::kFloat32).device(device));
                auto diff  = (oldest_pose_for_hess - mu_t).unsqueeze(1);
                auto prec_t = torch::zeros({3, 3},
                    torch::TensorOptions().dtype(torch::kFloat32).device(device));
                for (int r = 0; r < 3; r++)
                    for (int c = 0; c < 3; c++)
                        prec_t[r][c] = boundary_prior.precision(r, c);
                loss = loss + 0.5f * torch::matmul(diff.t(),
                                                    torch::matmul(prec_t, diff)).squeeze();
            }

            H = RoomConcept::autograd_hessian_3x3(loss, oldest_pose_for_hess);
        }
        else
        {
            // ── Degraded path: kinematic precision only ───────────────────────
            // H_obs is not included because the scan was contaminated (obstacle,
            // displacement confusion). Use only the motion-factor precision from the
            // next slot, which reflects purely the odometry model uncertainty.
            // This gives a conservative, direction-agnostic anchor.
            if (window.size() > 1)
            {
                auto prec_cpu = window[1].motion_prec_tensor.to(torch::kCPU);
                auto acc = prec_cpu.accessor<float, 2>();
                for (int r = 0; r < 3; r++)
                    for (int c = 0; c < 3; c++)
                        H(r, c) = acc[r][c];
            }
            else
            {
                // No motion link available: fall back to a weak isotropic prior.
                H = Eigen::Matrix3f::Identity() * params.eigenvalue_clamp_boundary;
            }
        }

        // ── Eigenvalue clamping (floor + ceiling) ─────────────────────────────
        // Floor prevents degenerate (zero-precision) directions.
        // Ceiling (eigenvalue_clamp_boundary_max) prevents over-confident priors
        // regardless of cause — acts as a safety net for both paths.
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(H);
        Eigen::Vector3f evals = eig.eigenvalues()
            .cwiseMax(params.eigenvalue_clamp_boundary)
            .cwiseMin(params.eigenvalue_clamp_boundary_max);
        H = eig.eigenvectors() * evals.asDiagonal() * eig.eigenvectors().transpose();

        boundary_prior.precision = H;
        boundary_prior.valid = true;
    }

} // namespace rc
