#pragma once

#include <vector>
#include <optional>
#include <chrono>
#include <Eigen/Dense>
#include "corner_visibility.h"

namespace rc
{

/**
 * EpistemicPlanner — Level 1: FIM-based information-gain target selection
 * =====================================================================
 * Generates candidate positions on a grid inside the room, scores them
 * using the Fisher-Information-Matrix D-optimality gain, exploration
 * distance bonus, and an Inhibition-of-Return (IoR) spatial visit grid,
 * then returns the best candidate as the planner's navigation target.
 *
 * Also handles angular-dominance detection (rotate-in-place), target
 * lifecycle (arrival, dwell timer), and visit-grid bookkeeping.
 */
class EpistemicPlanner
{
public:
    struct Params
    {
        float grid_resolution = 0.5f;        // spacing between candidate targets (m)
        float min_distance = 1.0f;           // ignore candidates closer than this to robot (m)
        int   max_candidates = 2000;         // cap on number of evaluated candidates
        float target_wall_margin = 1.0f;     // reject targets closer than this to walls (m)

        float angular_dominance_ratio = 50.0f; // σ²_θ / max(σ²_x, σ²_y) threshold
        float w_exploration  = 0.5f;         // weight: linear distance bonus (explore farther)

        // ---- Inhibition of Return (visit grid) ----
        float ior_cell_size   = 0.5f;        // spatial resolution of the visit grid (m)
        float ior_decay_time  = 120.0f;      // seconds until a visited cell is fully "stale" again
        float w_ior           = 2.0f;        // weight: staleness bonus

        // ---- FIM scoring ----
        float fim_corner_sigma  = 0.04f;     // isotropic corner detection noise σ (m)
        float fim_max_range     = 10.0f;     // max range for corner visibility (m)

        // ---- Target lifecycle ----
        float arrival_distance = 0.15f;      // target reached threshold (m)
        float dwell_time       = 2.0f;       // seconds to stop after reaching a target
    };

    /// Scored candidate target in room frame
    struct Target
    {
        Eigen::Vector2f position{0.f, 0.f};
        float score = 0.0f;
        float distance = 0.0f;
        float eigenvector_score = 0.0f;
        bool  rotate_in_place = false;
    };

    EpistemicPlanner();
    explicit EpistemicPlanner(Params params);

    // ---- State setters ----
    void set_room_bounds(const Eigen::Vector2f& min_corner, const Eigen::Vector2f& max_corner);
    void set_room_polygon(const std::vector<Eigen::Vector2f>& vertices);
    void set_robot_state(const Eigen::Affine2f& pose, const Eigen::Matrix3f& covariance);

    // ---- Target selection (public API) ----
    std::vector<Target> evaluate_targets() const;
    std::optional<Target> select_target() const;

    /// Called every plan cycle.  Returns the current navigation target,
    /// handling dwell and arrival logic internally.  Returns std::nullopt
    /// only when no valid target can be found.
    std::optional<Target> update_target();

    void clear_target() { current_target_.reset(); }
    const std::optional<Target>& current_target() const { return current_target_; }

    // ---- Accessors needed by Level 2 ----
    Eigen::Vector2f robot_pos() const { return robot_pose_.translation(); }
    float robot_theta() const { return std::atan2(robot_pose_.linear()(1,0), robot_pose_.linear()(0,0)); }
    const Eigen::Affine2f& robot_pose() const { return robot_pose_; }
    const Eigen::Matrix3f& robot_cov()  const { return robot_cov_; }
    const std::vector<Eigen::Vector2f>& room_corners() const { return room_corners_; }
    const Eigen::Vector2f& room_min() const { return room_min_; }
    const Eigen::Vector2f& room_max() const { return room_max_; }
    bool room_bounds_set() const { return room_bounds_set_; }

    Params params;

private:
    std::vector<Eigen::Vector2f> generate_candidates() const;
    bool is_angular_dominated() const;
    float score_fim_gain(const Eigen::Vector2f& candidate,
                         const Eigen::Matrix3f& prior_precision) const;

    // ---- State ----
    Eigen::Vector2f room_min_{0, 0};
    Eigen::Vector2f room_max_{0, 0};
    bool room_bounds_set_ = false;

    std::vector<Eigen::Vector2f> room_corners_;
    mutable std::vector<Eigen::Vector2f> cached_grid_;
    mutable bool grid_dirty_ = true;

    Eigen::Affine2f robot_pose_ = Eigen::Affine2f::Identity();
    Eigen::Matrix3f robot_cov_ = Eigen::Matrix3f::Identity();
    bool robot_state_set_ = false;

    // Persistent target
    std::optional<Target> current_target_;
    std::chrono::steady_clock::time_point dwell_until_{};
    bool dwelling_ = false;

    // ---- Inhibition of Return: spatial visit grid ----
    struct VisitGrid
    {
        std::vector<std::chrono::steady_clock::time_point> cells;
        int cols = 0, rows = 0;
        float cell_size = 0.5f;
        Eigen::Vector2f origin{0.f, 0.f};
        bool initialized = false;

        void init(const Eigen::Vector2f& room_min, const Eigen::Vector2f& room_max, float cs)
        {
            cell_size = cs;
            origin = room_min;
            cols = static_cast<int>(std::ceil((room_max.x() - room_min.x()) / cell_size));
            rows = static_cast<int>(std::ceil((room_max.y() - room_min.y()) / cell_size));
            cols = std::max(1, cols);
            rows = std::max(1, rows);
            cells.assign(cols * rows, std::chrono::steady_clock::time_point{});
            initialized = true;
        }

        int to_index(const Eigen::Vector2f& pos) const
        {
            int c = static_cast<int>((pos.x() - origin.x()) / cell_size);
            int r = static_cast<int>((pos.y() - origin.y()) / cell_size);
            c = std::clamp(c, 0, cols - 1);
            r = std::clamp(r, 0, rows - 1);
            return r * cols + c;
        }

        void mark_visited(const Eigen::Vector2f& pos)
        {
            if (!initialized) return;
            cells[to_index(pos)] = std::chrono::steady_clock::now();
        }

        float staleness(const Eigen::Vector2f& pos, float decay_s,
                        std::chrono::steady_clock::time_point now) const
        {
            if (!initialized) return 1.f;
            const auto& tp = cells[to_index(pos)];
            if (tp == std::chrono::steady_clock::time_point{}) return 1.f;
            const float elapsed = std::chrono::duration<float>(now - tp).count();
            return std::min(1.f, elapsed / std::max(0.1f, decay_s));
        }
    };
    VisitGrid visit_grid_;
};

} // namespace rc
