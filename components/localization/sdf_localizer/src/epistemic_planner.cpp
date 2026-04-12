#include "epistemic_planner.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>

namespace rc
{

EpistemicPlanner::EpistemicPlanner() : params{} {}
EpistemicPlanner::EpistemicPlanner(Params p) : params(std::move(p)) {}

// ---------------------------------------------------------------------------
// State setters
// ---------------------------------------------------------------------------
void EpistemicPlanner::set_room_bounds(const Eigen::Vector2f& min_corner,
                                       const Eigen::Vector2f& max_corner)
{
    room_min_ = min_corner;
    room_max_ = max_corner;
    room_bounds_set_ = true;
}

void EpistemicPlanner::set_room_polygon(const std::vector<Eigen::Vector2f>& vertices)
{
    room_corners_ = vertices;
}

void EpistemicPlanner::set_robot_state(const Eigen::Affine2f& pose,
                                       const Eigen::Matrix3f& covariance)
{
    robot_pose_ = pose;
    robot_cov_ = covariance;
    robot_state_set_ = true;
}

void EpistemicPlanner::set_sdf_eval(SdfEvalFn fn) { sdf_eval_fn_ = std::move(fn); }
void EpistemicPlanner::set_posterior_cov_eval(PosteriorCovFn fn) { posterior_cov_fn_ = std::move(fn); }
void EpistemicPlanner::set_lidar_obstacles(std::vector<Eigen::Vector2f> points) { lidar_obstacles_ = std::move(points); }

// ===========================================================================
// Stage 1 — Target selection (heuristic scoring, no EFE)
// ===========================================================================

// ---------------------------------------------------------------------------
// Candidate generation
// ---------------------------------------------------------------------------
std::vector<Eigen::Vector2f> EpistemicPlanner::generate_candidates() const
{
    if (!room_bounds_set_)
        return {};

    std::vector<Eigen::Vector2f> candidates;
    candidates.reserve(params.max_candidates);

    const float res = params.grid_resolution;
    const float min_d2 = params.min_distance * params.min_distance;
    const float max_d2 = params.max_distance * params.max_distance;

    for (float x = room_min_.x() + res * 0.5f; x < room_max_.x(); x += res)
    {
        for (float y = room_min_.y() + res * 0.5f; y < room_max_.y(); y += res)
        {
            const Eigen::Vector2f p{x, y};
            const float d2 = (p - robot_pos()).squaredNorm();
            if (d2 < min_d2 || d2 > max_d2)
                continue;
            candidates.emplace_back(p);
            if (static_cast<int>(candidates.size()) >= params.max_candidates)
                return candidates;
        }
    }
    return candidates;
}

// ---------------------------------------------------------------------------
// Phase 1a: uncertainty eigenvector decomposition
// ---------------------------------------------------------------------------
void EpistemicPlanner::decompose_uncertainty(Eigen::Vector2f& dominant_dir,
                                              float& eigenvalue_ratio) const
{
    const Eigen::Matrix2f pos_cov = robot_cov_.block<2,2>(0,0);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(pos_cov);
    const auto& vals = solver.eigenvalues();   // ascending
    const auto& vecs = solver.eigenvectors();
    dominant_dir = vecs.col(1).normalized();
    eigenvalue_ratio = (vals(0) > 1e-9f) ? vals(1) / vals(0) : 1.f;
}

float EpistemicPlanner::score_eigenvector_alignment(const Eigen::Vector2f& candidate,
                                                     const Eigen::Vector2f& dominant_dir) const
{
    const Eigen::Vector2f diff = candidate - robot_pos();
    const float dist = diff.norm();
    if (dist < 1e-3f) return 0.f;

    const float alignment = std::abs(diff.dot(dominant_dir)) / dist;

    Eigen::Vector2f unused;
    float ev_ratio;
    decompose_uncertainty(unused, ev_ratio);

    const float dist_factor = 1.f / (1.f + dist);
    return alignment * std::log1p(ev_ratio) * dist_factor;
}

// ---------------------------------------------------------------------------
// Phase 1a: corner proximity
// ---------------------------------------------------------------------------
float EpistemicPlanner::score_corner_proximity(const Eigen::Vector2f& candidate) const
{
    if (room_corners_.empty()) return 0.f;

    float min_dist = std::numeric_limits<float>::max();
    for (const auto& c : room_corners_)
        min_dist = std::min(min_dist, (candidate - c).norm());

    const float safe_dist = std::max(min_dist, params.grid_resolution);
    return 1.f / safe_dist;
}

// ---------------------------------------------------------------------------
// Phase 1b: angular dominance check
// ---------------------------------------------------------------------------
bool EpistemicPlanner::is_angular_dominated() const
{
    const float sigma2_theta = robot_cov_(2, 2);
    const float max_pos = std::max(robot_cov_(0, 0), robot_cov_(1, 1));
    if (max_pos < 1e-9f) return false;
    return (sigma2_theta / max_pos) > params.angular_dominance_ratio;
}

// ---------------------------------------------------------------------------
// Phase 2: SDF surprise
// ---------------------------------------------------------------------------
float EpistemicPlanner::score_sdf_surprise(const Eigen::Vector2f& candidate) const
{
    if (!sdf_eval_fn_) return 0.f;
    const float mean_abs_sdf = sdf_eval_fn_(candidate, robot_theta());
    return 1.f / (1.f + mean_abs_sdf);
}

// ---------------------------------------------------------------------------
// evaluate_targets — Stage 1 full pipeline (no EFE)
// ---------------------------------------------------------------------------
std::vector<EpistemicPlanner::Target> EpistemicPlanner::evaluate_targets() const
{
    if (!room_bounds_set_ || !robot_state_set_)
        return {};

    // Phase 1b: angular dominance → rotate in place
    if (is_angular_dominated())
    {
        Target rot;
        rot.position = robot_pos();
        rot.distance = 0.f;
        rot.rotate_in_place = true;
        rot.score = robot_cov_(2, 2);
        return {rot};
    }

    // Phase 1a: decompose position uncertainty
    Eigen::Vector2f dominant_dir;
    float ev_ratio;
    decompose_uncertainty(dominant_dir, ev_ratio);

    const auto candidates = generate_candidates();
    std::vector<Target> targets;
    targets.reserve(candidates.size());

    for (const auto& pos : candidates)
    {
        Target t;
        t.position = pos;
        t.distance = (pos - robot_pos()).norm();

        t.eigenvector_score  = score_eigenvector_alignment(pos, dominant_dir);
        t.corner_score       = score_corner_proximity(pos);
        t.sdf_surprise_score = score_sdf_surprise(pos);

        t.score = params.w_eigenvector  * t.eigenvector_score
                + params.w_corner       * t.corner_score
                + params.w_sdf_surprise * t.sdf_surprise_score;

        targets.push_back(t);
    }

    std::sort(targets.begin(), targets.end(),
              [](const Target& a, const Target& b) { return a.score > b.score; });

    return targets;
}

std::optional<EpistemicPlanner::Target> EpistemicPlanner::select_target() const
{
    auto targets = evaluate_targets();
    if (targets.empty())
        return std::nullopt;
    return targets.front();
}

// ===========================================================================
// Stage 2 — Policy sampling, rollout, EFE evaluation
// ===========================================================================

// ---------------------------------------------------------------------------
// sample_policies — generate constant-command policies toward a target
// ---------------------------------------------------------------------------
std::vector<EpistemicPlanner::Policy> EpistemicPlanner::sample_policies(const Target& target) const
{
    std::vector<Policy> policies;
    policies.reserve(params.num_policies + 1);

    // ---- Rotate-in-place: spread angular velocities evenly ----
    if (target.rotate_in_place)
    {
        const int n = std::max(params.num_policies, 2);
        for (int i = 0; i < n; ++i)
        {
            const float frac = static_cast<float>(i) / static_cast<float>(n - 1);
            const float rot = params.max_rot_speed * (2.f * frac - 1.f);
            Policy p;
            p.commands.resize(params.horizon_steps, {0.f, 0.f, rot});
            policies.push_back(std::move(p));
        }
        return policies;
    }

    // ---- Already at target: stop ----
    const Eigen::Vector2f to_target = target.position - robot_pos();
    const float dist = to_target.norm();
    if (dist < params.arrival_distance)
    {
        Policy p;
        p.commands.resize(params.horizon_steps, {0.f, 0.f, 0.f});
        policies.push_back(std::move(p));
        return policies;
    }

    // ---- Nominal velocity toward target ----
    const float horizon_time = static_cast<float>(params.horizon_steps) * params.dt;

    // Nominal heading: turn to face direction of travel
    // Convention: θ=0 → robot faces Y+. Forward dir = (-sinθ, cosθ).
    const float target_heading = std::atan2(-to_target.x(), to_target.y());
    float hdiff = target_heading - robot_theta();
    hdiff = std::atan2(std::sin(hdiff), std::cos(hdiff));
    const float nom_rot = std::clamp(hdiff / horizon_time,
                                     -params.max_rot_speed, params.max_rot_speed);

    // Scale forward speed by heading alignment: full speed when facing target,
    // zero when perpendicular or behind. Never drive backwards.
    const float alignment = std::max(0.f, std::cos(hdiff));   // 1 when aligned, 0 at ±π/2
    const float speed = std::min(dist / horizon_time, params.max_adv_speed) * alignment;
    const float nom_ax = 0.f;           // no lateral drift in nominal policy
    const float nom_ay = speed;         // always forward (body Y+)

    // (a) Nominal policy (zero noise)
    {
        Policy p;
        p.commands.resize(params.horizon_steps, {nom_ax, nom_ay, nom_rot});
        policies.push_back(std::move(p));
    }

    // (b) Noisy policies around nominal
    std::normal_distribution<float> noise_lin(0.f, params.policy_noise_lin);
    std::normal_distribution<float> noise_rot(0.f, params.policy_noise_rot);

    for (int i = 0; i < params.num_policies; ++i)
    {
        const float nx = noise_lin(rng_);
        const float ny = noise_lin(rng_);
        const float nr = noise_rot(rng_);

        const float ax = std::clamp(nom_ax  + nx, -params.max_adv_speed, params.max_adv_speed);
        const float ay = std::clamp(nom_ay  + ny, 0.f, params.max_adv_speed);  // never reverse
        const float ar = std::clamp(nom_rot + nr, -params.max_rot_speed, params.max_rot_speed);

        Policy p;
        p.commands.resize(params.horizon_steps, {ax, ay, ar});
        policies.push_back(std::move(p));
    }

    return policies;
}

// ---------------------------------------------------------------------------
// rollout_policy — kinematic integration through the horizon
//   dx = (adv_x·cos θ − adv_y·sin θ) · dt
//   dy = (adv_x·sin θ + adv_y·cos θ) · dt
//   dθ = rot · dt
// ---------------------------------------------------------------------------
void EpistemicPlanner::rollout_policy(Policy& policy) const
{
    policy.predicted_states.clear();
    policy.predicted_states.reserve(params.horizon_steps + 1);

    Eigen::Vector3f s{robot_pos().x(), robot_pos().y(), robot_theta()};
    policy.predicted_states.push_back(s);

    for (int t = 0; t < params.horizon_steps; ++t)
    {
        const auto& c = policy.commands[t];
        const float c_t = std::cos(s[2]);
        const float s_t = std::sin(s[2]);

        s[0] += (c.adv_x * c_t - c.adv_y * s_t) * params.dt;
        s[1] += (c.adv_x * s_t + c.adv_y * c_t) * params.dt;
        s[2] += c.rot * params.dt;
        s[2]  = std::atan2(std::sin(s[2]), std::cos(s[2]));   // normalise

        policy.predicted_states.push_back(s);
    }
}

// ---------------------------------------------------------------------------
// evaluate_policy_efe — Expected Free Energy for a policy
//   G(π) = -w_e · epistemic + w_p · Σ_t ||pos_t − target||²  + boundary
// epistemic ≈ 0.5·(log|Σ_prior| − log|Σ_posterior(s_H)|)  at final state
// ---------------------------------------------------------------------------
void EpistemicPlanner::evaluate_policy_efe(Policy& policy, const Target& target) const
{
    float pragmatic = 0.f;
    float boundary  = 0.f;
    float obstacle  = 0.f;
    float epistemic = 0.f;

    const float log_det_prior = std::log(std::max(robot_cov_.determinant(), 1e-20f));
    const float obs_r2 = params.obstacle_radius * params.obstacle_radius;

    for (std::size_t t = 1; t < policy.predicted_states.size(); ++t)
    {
        const auto& s = policy.predicted_states[t];
        const Eigen::Vector2f pos{s[0], s[1]};

        // Pragmatic: squared distance to preferred state (target)
        pragmatic += (pos - target.position).squaredNorm();

        // Boundary violation penalty
        if (room_bounds_set_ &&
            (pos.x() < room_min_.x() || pos.x() > room_max_.x() ||
             pos.y() < room_min_.y() || pos.y() > room_max_.y()))
        {
            boundary += params.w_boundary;
        }

        // Obstacle avoidance: repulsive potential from nearby lidar floor points
        for (const auto& lp : lidar_obstacles_)
        {
            const float d2 = (pos - lp).squaredNorm();
            if (d2 < obs_r2 && d2 > 1e-6f)
                obstacle += params.obstacle_k / d2;
        }
    }

    // Epistemic: mutual information at the final predicted state
    if (posterior_cov_fn_ && policy.predicted_states.size() > 1)
    {
        const auto& sf = policy.predicted_states.back();
        const Eigen::Matrix3f post_cov = posterior_cov_fn_({sf[0], sf[1]}, sf[2]);
        const float log_det_post = std::log(std::max(post_cov.determinant(), 1e-20f));
        epistemic = 0.5f * (log_det_prior - log_det_post);
    }

    policy.epistemic_value = epistemic;
    policy.pragmatic_value = pragmatic;

    // Minimise G: maximise information gain AND minimise distance to target
    policy.efe = -params.w_epistemic * epistemic
                + params.w_pragmatic * pragmatic
                + params.w_obstacle  * obstacle
                + boundary;
}

// ---------------------------------------------------------------------------
// plan — orchestrate target selection → policy sampling → EFE → first action
// ---------------------------------------------------------------------------
std::optional<EpistemicPlanner::PlanResult> EpistemicPlanner::plan()
{
    // Check if we are dwelling (stopped after reaching a target)
    if (dwelling_)
    {
        if (std::chrono::steady_clock::now() < dwell_until_)
            return PlanResult{.command = ControlCommand{}, .target = current_target_.value_or(Target{}),
                              .best_policy = {}, .valid = true};
        dwelling_ = false;
        current_target_.reset();
    }

    // Check if current target is reached or needs refreshing
    if (current_target_.has_value() && !current_target_->rotate_in_place)
    {
        const float dist = (current_target_->position - robot_pos()).norm();
        if (dist < params.arrival_distance)
        {
            dwelling_ = true;
            dwell_until_ = std::chrono::steady_clock::now()
                         + std::chrono::milliseconds(static_cast<int>(params.dwell_time * 1000.f));
            return PlanResult{.command = ControlCommand{}, .target = *current_target_,
                              .best_policy = {}, .valid = true};
        }
    }

    // Select a new target only when we don't have one
    if (!current_target_.has_value())
    {
        auto target_opt = select_target();
        if (!target_opt)
            return std::nullopt;
        current_target_ = *target_opt;
    }

    const auto& target = *current_target_;
    ControlCommand cmd{};

    if (target.rotate_in_place)
    {
        cmd.rot = params.max_rot_speed * 0.5f;
    }
    else
    {
        // 1. Convert target to robot body frame using the inverse of robot_pose_
        const Eigen::Vector2f target_in_body = robot_pose_.inverse() * target.position;

        // 2. Angle error in body frame: atan2(-x, y) gives CCW+ signed angle from Y+ axis
        const float angle_error = std::atan2(-target_in_body.x(), target_in_body.y());

        // Proportional rotation
        cmd.rot = std::clamp(params.k_rot * angle_error, -params.max_rot_speed, params.max_rot_speed);

        // 3. Forward speed: max_adv_speed * gaussian(angle_error)
        //    Full speed when aligned, decays smoothly as heading error grows
        const float adv = params.max_adv_speed * std::exp(-0.5f * angle_error * angle_error
                                                          / (params.gaussian_sigma * params.gaussian_sigma));
        cmd.adv_y = adv;    // forward (body Y+)
        cmd.adv_x = 0.f;    // no lateral

    }

    return PlanResult{
        .command     = cmd,
        .target      = target,
        .best_policy = {},
        .valid       = true
    };
}

} // namespace rc
