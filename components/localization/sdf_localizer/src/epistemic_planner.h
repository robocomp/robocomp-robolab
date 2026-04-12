#pragma once

#include <vector>
#include <optional>
#include <functional>
#include <chrono>
#include <random>
#include <Eigen/Dense>

namespace rc
{

/**
 * EpistemicPlanner
 * ================
 * Active-inference action layer with a two-stage pipeline:
 *
 * Stage 1 — Target selection (fast heuristics, no EFE):
 *  1a. Geometric: dominant uncertainty eigenvector + corner proximity.
 *  1b. Angular: detect heading-dominated uncertainty → rotate-in-place.
 *  2.  SDF surprise: predicted mean |SDF| at each candidate.
 *
 * Stage 2 — Policy evaluation (Expected Free Energy):
 *  Sample action policies (constant-command control sequences) toward the
 *  selected target, rollout their predicted trajectories via the kinematic
 *  model, evaluate each policy's EFE:
 *      G(π) = -w_e · epistemic(π) + w_p · pragmatic(π) + boundary(π)
 *  where epistemic = mutual information at the final predicted state and
 *  pragmatic = accumulated squared distance to the preferred state (target).
 *  The first control command of the minimum-EFE policy is returned.
 */
class EpistemicPlanner
{
public:
    struct Params
    {
        // ---- Stage 1: target selection ----
        float grid_resolution = 0.25f;       // spacing between candidate targets (m)
        float min_distance = 0.3f;           // ignore candidates closer than this to robot (m)
        float max_distance = 5.0f;           // ignore candidates farther than this from robot (m)
        int   max_candidates = 200;          // cap on number of evaluated candidates

        float w_eigenvector = 1.0f;          // weight: alignment with dominant uncertainty axis
        float w_corner      = 0.5f;          // weight: proximity to room corners
        float angular_dominance_ratio = 50.0f; // σ²_θ / max(σ²_x, σ²_y) threshold
        float w_sdf_surprise = 2.0f;         // weight: SDF surprise score

        // ---- Stage 2: policy sampling & EFE ----
        int   num_policies   = 30;           // number of sampled policies (+ 1 nominal)
        int   horizon_steps  = 10;           // planning horizon steps
        float dt             = 0.2f;         // seconds per horizon step
        float max_adv_speed  = 0.3f;         // max translational speed (m/s)
        float max_rot_speed  = 0.5f;         // max angular speed (rad/s)
        float policy_noise_lin = 0.1f;       // std dev of linear velocity noise (m/s)
        float policy_noise_rot = 0.2f;       // std dev of angular velocity noise (rad/s)
        float w_epistemic    = 1.0f;         // EFE weight: information gain
        float w_pragmatic    = 1.0f;         // EFE weight: distance to preferred state
        float w_boundary     = 10.0f;        // penalty per step outside room bounds
        float arrival_distance = 0.15f;      // target reached threshold (m)
        float dwell_time = 2.0f;                // seconds to stop after reaching a target

        // ---- Simple angle-distance controller gains ----
        float k_rot  = 2.0f;                   // proportional gain for heading error (rad/s per rad)
        float gaussian_sigma = 0.5f;            // σ for gaussian speed profile (rad)

        // ---- Obstacle avoidance (lidar floor projection) ----
        float obstacle_radius  = 0.5f;         // interaction distance around each lidar point (m)
        float obstacle_k       = 1.0f;         // repulsive potential gain: k / d²
        float w_obstacle       = 5.0f;         // EFE weight for obstacle cost
    };

    /// Control command in robot body frame (adv_x = lateral, adv_y = forward)
    struct ControlCommand
    {
        float adv_x = 0.f;   // lateral speed (m/s)
        float adv_y = 0.f;   // forward speed (m/s)
        float rot   = 0.f;   // angular speed (rad/s)
    };

    /// Scored candidate target in room frame (Stage 1 output)
    struct Target
    {
        Eigen::Vector2f position{0.f, 0.f};  // room-frame (x, y)
        float score = 0.0f;                  // composite heuristic score (higher = better)
        float distance = 0.0f;               // distance from current robot position

        // Sub-scores (for diagnostics)
        float eigenvector_score = 0.0f;
        float corner_score = 0.0f;
        float sdf_surprise_score = 0.0f;
        bool  rotate_in_place = false;       // heading uncertainty dominates → rotate only
    };

    /// A sampled action policy with its predicted trajectory and EFE
    struct Policy
    {
        std::vector<ControlCommand> commands;              // one per horizon step
        std::vector<Eigen::Vector3f> predicted_states;     // [x, y, θ] per step (size = horizon+1)
        float efe             = 0.f;                       // total expected free energy (lower = better)
        float epistemic_value = 0.f;                       // information gain component
        float pragmatic_value = 0.f;                       // goal-distance component
    };

    /// Result of plan(): the first control command to execute
    struct PlanResult
    {
        ControlCommand command;       // first action of the best policy
        Target         target;        // the selected preferred state
        Policy         best_policy;   // winning policy (for visualisation)
        bool           valid = false;
    };

    /// Callback: (candidate_xy, robot_theta) → mean |SDF| at that pose.
    using SdfEvalFn = std::function<float(const Eigen::Vector2f&, float)>;

    /// Callback: (position, theta) → 3×3 predicted posterior covariance.
    using PosteriorCovFn = std::function<Eigen::Matrix3f(const Eigen::Vector2f&, float)>;

    EpistemicPlanner();
    explicit EpistemicPlanner(Params params);

    // ---- State setters ----
    void set_room_bounds(const Eigen::Vector2f& min_corner, const Eigen::Vector2f& max_corner);
    void set_room_polygon(const std::vector<Eigen::Vector2f>& vertices);
    void set_robot_state(const Eigen::Affine2f& pose, const Eigen::Matrix3f& covariance);
    void set_sdf_eval(SdfEvalFn fn);
    void set_posterior_cov_eval(PosteriorCovFn fn);

    /// Set floor-projected lidar points (room frame) for obstacle avoidance.
    void set_lidar_obstacles(std::vector<Eigen::Vector2f> points);

    /// Main entry: select target → drive toward it → return command.
    std::optional<PlanResult> plan();

    /// Clear the current target (e.g. when self-targeting is toggled off).
    void clear_target() { current_target_.reset(); }

    /// Stage 1 only: select best target by heuristic scoring (no EFE).
    std::optional<Target> select_target() const;

    /// Stage 1: generate and score all candidate targets (sorted, highest score first).
    std::vector<Target> evaluate_targets() const;

    Params params;

private:
    // ---- Stage 1: target selection ----
    std::vector<Eigen::Vector2f> generate_candidates() const;
    void decompose_uncertainty(Eigen::Vector2f& dominant_dir, float& eigenvalue_ratio) const;
    float score_eigenvector_alignment(const Eigen::Vector2f& candidate,
                                      const Eigen::Vector2f& dominant_dir) const;
    float score_corner_proximity(const Eigen::Vector2f& candidate) const;
    bool is_angular_dominated() const;
    float score_sdf_surprise(const Eigen::Vector2f& candidate) const;

    // ---- Stage 2: policy sampling & EFE ----
    std::vector<Policy> sample_policies(const Target& target) const;
    void rollout_policy(Policy& policy) const;
    void evaluate_policy_efe(Policy& policy, const Target& target) const;

    // ---- State ----
    Eigen::Vector2f room_min_{0, 0};
    Eigen::Vector2f room_max_{0, 0};
    bool room_bounds_set_ = false;

    std::vector<Eigen::Vector2f> room_corners_;

    Eigen::Affine2f robot_pose_ = Eigen::Affine2f::Identity();
    Eigen::Matrix3f robot_cov_ = Eigen::Matrix3f::Identity();
    bool robot_state_set_ = false;

    // Convenience accessors
    Eigen::Vector2f robot_pos() const { return robot_pose_.translation(); }
    float robot_theta() const { return std::atan2(robot_pose_.linear()(1,0), robot_pose_.linear()(0,0)); }

    SdfEvalFn sdf_eval_fn_;
    PosteriorCovFn posterior_cov_fn_;
    std::vector<Eigen::Vector2f> lidar_obstacles_;   // floor-projected lidar points (room frame)
    mutable std::mt19937 rng_{42};

    // Persistent target in room frame (fixed until arrived or cleared)
    std::optional<Target> current_target_;

    // Dwell timer: robot stops for dwell_time seconds after reaching a target
    std::chrono::steady_clock::time_point dwell_until_{};
    bool dwelling_ = false;
};

} // namespace rc
