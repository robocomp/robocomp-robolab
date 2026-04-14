#include "epistemic_controller.h"
#include <algorithm>
#include <cmath>

namespace rc
{

EpistemicController::EpistemicController() : params{} {}
EpistemicController::EpistemicController(Params planner_params, EpistemicPlanner::Params selector_params)
    : params(std::move(planner_params)), epistemic_planner_(std::move(selector_params)) {}

// ---------------------------------------------------------------------------
// State setters — forwarded to EpistemicPlanner + local obstacle state
// ---------------------------------------------------------------------------
void EpistemicController::set_room_bounds(const Eigen::Vector2f& min_corner,
                                       const Eigen::Vector2f& max_corner)
{
    epistemic_planner_.set_room_bounds(min_corner, max_corner);
}

void EpistemicController::set_room_polygon(const std::vector<Eigen::Vector2f>& vertices)
{
    epistemic_planner_.set_room_polygon(vertices);

    // Pre-compute edge segment data for wall filtering
    edge_segments_.clear();
    edge_segments_.reserve(vertices.size());
    for (std::size_t i = 0; i < vertices.size(); ++i)
    {
        const auto& a = vertices[i];
        const Eigen::Vector2f ab = vertices[(i + 1) % vertices.size()] - a;
        edge_segments_.push_back({a, ab, ab.squaredNorm()});
    }
}

void EpistemicController::set_robot_state(const Eigen::Affine2f& pose,
                                       const Eigen::Matrix3f& covariance)
{
    epistemic_planner_.set_robot_state(pose, covariance);
}

void EpistemicController::set_lidar_obstacles(std::vector<Eigen::Vector2f> points)
{
    if (!edge_segments_.empty())
    {
        const float margin2 = params.wall_filter_margin * params.wall_filter_margin;
        std::vector<Eigen::Vector2f> filtered;
        filtered.reserve(points.size());

        for (const auto& p : points)
        {
            bool near_wall = false;
            for (const auto& e : edge_segments_)
            {
                const float t = std::clamp((p - e.a).dot(e.ab) / e.ab_sq_norm, 0.f, 1.f);
                const float d2 = (p - (e.a + t * e.ab)).squaredNorm();
                if (d2 < margin2) { near_wall = true; break; }
            }
            if (!near_wall)
                filtered.push_back(p);
        }
        lidar_obstacles_ = std::move(filtered);
    }
    else
    {
        lidar_obstacles_ = std::move(points);
    }
}

// ===========================================================================
// Reactive speed governor — localization quality
// ===========================================================================
void EpistemicController::set_localization_quality(float sdf_mse)
{
    const float range = params.sdf_danger - params.sdf_safe;
    if (range < 1e-9f)
        governor_alpha_ = 1.f;
    else
        governor_alpha_ = std::clamp(1.f - (sdf_mse - params.sdf_safe) / range,
                                     params.governor_alpha_min, 1.f);
}

// ===========================================================================
// Perceptual-bandwidth speed limit
// ===========================================================================
EpistemicController::ControlCommand
EpistemicController::apply_speed_limit(ControlCommand cmd) const
{
    // Governor: scale both limits by localization quality
    const float eff_rot_max = params.max_rot_speed * governor_alpha_;
    const float eff_adv_max = params.max_adv_speed * governor_alpha_;

    cmd.rot = std::clamp(cmd.rot, -eff_rot_max, eff_rot_max);
    const float rot_frac = std::abs(cmd.rot) / std::max(1e-6f, eff_rot_max);
    const float eff_max  = eff_adv_max
                         * std::max(0.f, 1.f - params.bandwidth_coupling * rot_frac);
    const float adv_norm = std::sqrt(cmd.adv_x * cmd.adv_x + cmd.adv_y * cmd.adv_y);
    if (adv_norm > eff_max && adv_norm > 1e-6f)
    {
        const float scale = eff_max / adv_norm;
        cmd.adv_x *= scale;
        cmd.adv_y *= scale;
    }
    return cmd;
}

// ===========================================================================
// Level 2 — Arc trajectory generation
// ===========================================================================
std::vector<EpistemicController::Policy>
EpistemicController::generate_arc_policies(const EpistemicPlanner::Target& target) const
{
    std::vector<Policy> policies;
    const int K = params.num_arc_curvatures;

    // ---- Rotate-in-place target ----
    if (target.rotate_in_place)
    {
        for (int i = 0; i < std::max(K, 2); ++i)
        {
            const float frac = static_cast<float>(i) / std::max(1.f, static_cast<float>(K - 1));
            const float rot  = params.max_rot_speed * (2.f * frac - 1.f);
            Policy p;
            p.commands.resize(params.horizon_steps, {0.f, 0.f, rot});
            policies.push_back(std::move(p));
        }
        return policies;
    }

    // ---- Target-seeking nominal command ----
    const Eigen::Vector2f tb = epistemic_planner_.robot_pose().inverse() * target.position;
    const float angle_err  = std::atan2(-tb.x(), tb.y());
    const float dist       = tb.norm();
    const float hor_time   = static_cast<float>(params.horizon_steps) * params.dt;
    const float alignment  = std::max(0.f, std::cos(angle_err));
    const float nom_speed  = std::min(dist / hor_time, params.max_adv_speed) * alignment;
    const float nom_rot    = std::clamp(params.k_rot * angle_err,
                                         -params.max_rot_speed, params.max_rot_speed);

    // ---- K arcs: spread rotation around nominal ----
    for (int i = 0; i < K; ++i)
    {
        const float frac = (K > 1)
            ? 2.f * static_cast<float>(i) / static_cast<float>(K - 1) - 1.f
            : 0.f;

        const float rot = std::clamp(nom_rot + frac * params.max_rot_speed,
                                      -params.max_rot_speed, params.max_rot_speed);
        const float speed = std::max(0.f, nom_speed * (1.f - 0.5f * std::abs(frac)));

        const auto arc_cmd = apply_speed_limit({0.f, speed, rot});
        Policy p;
        p.commands.resize(params.horizon_steps, arc_cmd);
        policies.push_back(std::move(p));
    }

    // ---- Pure rotations ----
    for (float s : {-1.f, -0.5f, 0.5f, 1.f})
    {
        Policy p;
        p.commands.resize(params.horizon_steps, {0.f, 0.f, s * params.max_rot_speed});
        policies.push_back(std::move(p));
    }

    return policies;
}

// ===========================================================================
// Kinematic rollout
// ===========================================================================
void EpistemicController::rollout_policy(Policy& policy) const
{
    policy.predicted_states.clear();
    policy.predicted_states.reserve(params.horizon_steps + 1);

    Eigen::Vector3f s{epistemic_planner_.robot_pos().x(),
                      epistemic_planner_.robot_pos().y(),
                      epistemic_planner_.robot_theta()};
    policy.predicted_states.push_back(s);

    for (int t = 0; t < params.horizon_steps; ++t)
    {
        const auto& c = policy.commands[t];
        const float c_t = std::cos(s[2]);
        const float s_t = std::sin(s[2]);

        s[0] += (c.adv_x * c_t - c.adv_y * s_t) * params.dt;
        s[1] += (c.adv_x * s_t + c.adv_y * c_t) * params.dt;
        s[2] += c.rot * params.dt;
        s[2]  = std::atan2(std::sin(s[2]), std::cos(s[2]));

        policy.predicted_states.push_back(s);
    }
}

// ===========================================================================
// EFE evaluation
// ===========================================================================
void EpistemicController::evaluate_policy_efe(Policy& policy,
                                            const EpistemicPlanner::Target& target,
                                            const Eigen::Matrix3f& prior_precision) const
{
    float pragmatic = 0.f;
    float boundary  = 0.f;
    float obstacle  = 0.f;
    float epistemic = 0.f;

    const float obs_r2 = params.obstacle_radius * params.obstacle_radius;
    const auto& room_corners = epistemic_planner_.room_corners();
    const bool have_polygon = !room_corners.empty();

    for (std::size_t t = 1; t < policy.predicted_states.size(); ++t)
    {
        const auto& s = policy.predicted_states[t];
        const Eigen::Vector2f pos{s[0], s[1]};

        pragmatic += (pos - target.position).squaredNorm();

        const Eigen::Vector2f diff = target.position - pos;
        if (diff.squaredNorm() > 0.01f)
        {
            const float ct = std::cos(s[2]);
            const float st = std::sin(s[2]);
            const float bx =  ct * diff.x() + st * diff.y();
            const float by = -st * diff.x() + ct * diff.y();
            const float heading_err = std::atan2(-bx, by);
            pragmatic += params.w_heading * heading_err * heading_err;
        }

        if (have_polygon)
        {
            if (!corner_visibility::point_in_polygon(pos, room_corners))
                boundary += params.w_boundary;
        }
        else if (epistemic_planner_.room_bounds_set() &&
                 (pos.x() < epistemic_planner_.room_min().x() || pos.x() > epistemic_planner_.room_max().x() ||
                  pos.y() < epistemic_planner_.room_min().y() || pos.y() > epistemic_planner_.room_max().y()))
        {
            boundary += params.w_boundary;
        }

        float step_obs = 0.f;
        for (const auto& lp : lidar_obstacles_)
        {
            const float d2 = (pos - lp).squaredNorm();
            if (d2 < obs_r2 && d2 > 1e-6f)
                step_obs += params.obstacle_k / d2;
        }
        obstacle += std::min(step_obs, params.obstacle_step_cap);
    }

    // Epistemic: FIM-based information gain at the final predicted state
    if (have_polygon && policy.predicted_states.size() > 1)
    {
        const auto& sf = policy.predicted_states.back();
        const Eigen::Vector2f final_pos{sf[0], sf[1]};
        const float final_theta = sf[2];

        auto vis = corner_visibility::visible_corners(final_pos, room_corners,
                                                       params.fim_max_range);
        if (!vis.empty())
        {
            const auto fim = corner_visibility::corner_fim(final_pos, final_theta, vis,
                                                            room_corners, params.fim_corner_sigma);
            epistemic = corner_visibility::d_optimality_gain(prior_precision, fim);
        }
    }

    policy.epistemic_value = epistemic;
    policy.pragmatic_value = pragmatic;
    policy.obstacle_value = obstacle;
    policy.boundary_value = boundary;

    policy.efe = -params.w_epistemic * epistemic
                + params.w_pragmatic * pragmatic
                + params.w_obstacle  * obstacle
                + boundary;
}

// ===========================================================================
// plan — orchestrates Level 1 + Level 2
// ===========================================================================
std::optional<EpistemicController::PlanResult> EpistemicController::plan()
{
    // ---- Level 1: update / select target ----
    auto target_opt = epistemic_planner_.update_target();
    if (!target_opt)
        return std::nullopt;

    // Dwell: if target selector is dwelling, return stop command
    // (update_target returns the current target during dwell)
    const auto& target = *target_opt;

    // ---- Level 2: generate arcs, rollout, evaluate EFE, pick best ----
    const Eigen::Matrix3f reg_cov = epistemic_planner_.robot_cov() + 1e-6f * Eigen::Matrix3f::Identity();
    const Eigen::Matrix3f prior_precision = reg_cov.inverse();

    auto policies = generate_arc_policies(target);
    for (auto& p : policies)
    {
        rollout_policy(p);
        evaluate_policy_efe(p, target, prior_precision);
    }

    auto best = std::min_element(policies.begin(), policies.end(),
        [](const Policy& a, const Policy& b) { return a.efe < b.efe; });

    if (best != policies.end() && !best->commands.empty())
    {
        auto best_policy = *best;
        const auto final_cmd = apply_speed_limit(best_policy.commands.front());

        return PlanResult{
            .command       = final_cmd,
            .target        = target,
            .best_policy   = std::move(best_policy),
            .all_policies  = std::move(policies),
            .valid         = true
        };
    }

    // ---- Fallback: simple proportional controller ----
    ControlCommand cmd{};
    if (target.rotate_in_place)
    {
        cmd.rot = params.max_rot_speed * 0.5f;
    }
    else
    {
        const Eigen::Vector2f tb = epistemic_planner_.robot_pose().inverse() * target.position;
        const float ae = std::atan2(-tb.x(), tb.y());
        cmd.rot   = std::clamp(params.k_rot * ae, -params.max_rot_speed, params.max_rot_speed);
        cmd.adv_y = params.max_adv_speed
                  * std::exp(-0.5f * ae * ae / (params.gaussian_sigma * params.gaussian_sigma));
    }
    return PlanResult{.command = apply_speed_limit(cmd), .target = target, .best_policy = {}, .valid = true};
}

} // namespace rc
