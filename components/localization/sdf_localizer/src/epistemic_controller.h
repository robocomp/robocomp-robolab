#pragma once

#include <vector>
#include <optional>
#include <Eigen/Dense>
#include "epistemic_planner.h"

namespace rc
{

/**
 * EpistemicController — Level 2: Arc trajectory EFE evaluation
 * ==========================================================
 * Given a navigation target from EpistemicPlanner (Level 1), generates
 * constant-curvature arc policies, rolls them out via a kinematic model,
 * evaluates each policy's Expected Free Energy (EFE), and returns the
 * first control command of the best policy.
 *
 *   G(π) = −w_e · epistemic(π) + w_p · pragmatic(π) + w_o · obstacle(π) + boundary(π)
 */
class EpistemicController
{
public:
    struct Params
    {
        // ---- Arc trajectory generation ----
        int   num_arc_curvatures = 9;        // discrete curvatures (odd ⟹ includes κ=0)

        // ---- Policy rollout & EFE ----
        int   horizon_steps  = 20;
        float dt             = 0.2f;
        float max_adv_speed  = 0.9f;
        float max_rot_speed  = 0.75f;
        float w_epistemic    = 1.0f;
        float w_pragmatic    = 1.0f;
        float w_heading      = 2.0f;
        float w_boundary     = 10.0f;

        // ---- Simple angle-distance controller gains ----
        float k_rot  = 2.0f;
        float gaussian_sigma = 0.5f;

        // ---- Obstacle avoidance (lidar floor projection) ----
        float obstacle_radius  = 0.35f;
        float obstacle_k       = 0.5f;
        float obstacle_step_cap = 10.0f;
        float w_obstacle       = 2.0f;
        float wall_filter_margin = 0.30f;

        // ---- Perceptual bandwidth speed limit ----
        // Couples rotation and translation so that sharp turns reduce the
        // allowed translation speed.  Models a finite perceptual bandwidth:
        //   effective_max_adv = max_adv_speed × (1 − bandwidth_coupling × |ω|/max_rot_speed)
        float bandwidth_coupling = 0.7f;

        // ---- Reactive speed governor (localization quality) ----
        // Scales both v_max and ω_max when SDF-MSE exceeds a safe threshold.
        //   α = clamp(1 − (sdf_mse − sdf_safe) / (sdf_danger − sdf_safe), α_min, 1)
        float sdf_safe    = 0.04f;       // below this SDF-MSE → full speed
        float sdf_danger  = 0.08f;       // at/above this → minimum speed
        float governor_alpha_min = 0.2f;  // minimum speed fraction

        // ---- FIM scoring at final state (epistemic EFE term) ----
        float fim_corner_sigma  = 0.04f;
        float fim_max_range     = 10.0f;
    };

    /// Control command in robot body frame
    struct ControlCommand
    {
        float adv_x = 0.f;
        float adv_y = 0.f;
        float rot   = 0.f;
    };

    /// A sampled action policy with its predicted trajectory and EFE
    struct Policy
    {
        std::vector<ControlCommand> commands;
        std::vector<Eigen::Vector3f> predicted_states;
        float efe             = 0.f;
        float epistemic_value = 0.f;
        float pragmatic_value = 0.f;
        float obstacle_value  = 0.f;
        float boundary_value  = 0.f;
    };

    /// Result of plan()
    struct PlanResult
    {
        ControlCommand command;
        EpistemicPlanner::Target target;
        Policy         best_policy;
        std::vector<Policy> all_policies;
        bool           valid = false;
    };

    EpistemicController();
    explicit EpistemicController(Params planner_params, EpistemicPlanner::Params selector_params = {});

    // ---- State setters (forwarded to EpistemicPlanner) ----
    void set_room_bounds(const Eigen::Vector2f& min_corner, const Eigen::Vector2f& max_corner);
    void set_room_polygon(const std::vector<Eigen::Vector2f>& vertices);
    void set_robot_state(const Eigen::Affine2f& pose, const Eigen::Matrix3f& covariance);
    void set_lidar_obstacles(std::vector<Eigen::Vector2f> points);
    void set_localization_quality(float sdf_mse);

    /// Main entry: select target (Level 1) → drive toward it (Level 2) → return command.
    std::optional<PlanResult> plan();

    void clear_target() { epistemic_planner_.clear_target(); }

    float governor_alpha() const { return governor_alpha_; }

    /// Access the Level 1 epistemic planner (e.g. for evaluate_targets()).
    EpistemicPlanner&       epistemic_planner()       { return epistemic_planner_; }
    const EpistemicPlanner& epistemic_planner() const { return epistemic_planner_; }

    Params params;

private:
    // ---- Level 2 internals ----
    std::vector<Policy> generate_arc_policies(const EpistemicPlanner::Target& target) const;
    void rollout_policy(Policy& policy) const;
    void evaluate_policy_efe(Policy& policy, const EpistemicPlanner::Target& target,
                             const Eigen::Matrix3f& prior_precision) const;
    ControlCommand apply_speed_limit(ControlCommand cmd) const;

    // ---- Pre-computed edge segment data (set by set_room_polygon) ----
    struct EdgeSegment { Eigen::Vector2f a, ab; float ab_sq_norm; };
    std::vector<EdgeSegment> edge_segments_;

    // ---- Composition ----
    EpistemicPlanner epistemic_planner_;
    std::vector<Eigen::Vector2f> lidar_obstacles_;
    float governor_alpha_ = 1.f;   // current speed-governor scaling factor
};

} // namespace rc
