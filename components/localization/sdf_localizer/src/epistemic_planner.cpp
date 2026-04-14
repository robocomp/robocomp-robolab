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
    grid_dirty_ = true;
    if (!visit_grid_.initialized)
        visit_grid_.init(room_min_, room_max_, params.ior_cell_size);
}

void EpistemicPlanner::set_room_polygon(const std::vector<Eigen::Vector2f>& vertices)
{
    room_corners_ = vertices;
    grid_dirty_ = true;
}

void EpistemicPlanner::set_robot_state(const Eigen::Affine2f& pose,
                                      const Eigen::Matrix3f& covariance)
{
    robot_pose_ = pose;
    robot_cov_ = covariance;
    robot_state_set_ = true;
}

// ===========================================================================
// Candidate generation
// ===========================================================================
std::vector<Eigen::Vector2f> EpistemicPlanner::generate_candidates() const
{
    if (!room_bounds_set_)
        return {};

    // Rebuild static grid cache when room geometry changes
    if (grid_dirty_)
    {
        cached_grid_.clear();
        const float res = params.grid_resolution;
        const float wm2 = params.target_wall_margin * params.target_wall_margin;

        for (float x = room_min_.x() + res * 0.5f; x < room_max_.x(); x += res)
        {
            for (float y = room_min_.y() + res * 0.5f; y < room_max_.y(); y += res)
            {
                const Eigen::Vector2f p{x, y};
                if (!room_corners_.empty() &&
                    !corner_visibility::point_in_polygon(p, room_corners_))
                    continue;
                if (!room_corners_.empty())
                {
                    bool too_close = false;
                    for (std::size_t i = 0; i < room_corners_.size(); ++i)
                    {
                        const auto& a = room_corners_[i];
                        const auto& b = room_corners_[(i + 1) % room_corners_.size()];
                        const Eigen::Vector2f ab = b - a;
                        const float t = std::clamp((p - a).dot(ab) / ab.squaredNorm(), 0.f, 1.f);
                        if ((p - (a + t * ab)).squaredNorm() < wm2) { too_close = true; break; }
                    }
                    if (too_close) continue;
                }
                cached_grid_.emplace_back(p);
            }
        }
        grid_dirty_ = false;
    }

    // Filter cached grid by min_distance from robot (dynamic per cycle)
    const float min_d2 = params.min_distance * params.min_distance;
    std::vector<Eigen::Vector2f> candidates;
    candidates.reserve(std::min(static_cast<int>(cached_grid_.size()), params.max_candidates));

    for (const auto& p : cached_grid_)
    {
        if ((p - robot_pos()).squaredNorm() < min_d2)
            continue;
        candidates.emplace_back(p);
        if (static_cast<int>(candidates.size()) >= params.max_candidates)
            break;
    }
    return candidates;
}

// ===========================================================================
// Angular dominance check
// ===========================================================================
bool EpistemicPlanner::is_angular_dominated() const
{
    const float sigma2_theta = robot_cov_(2, 2);
    const float max_pos = std::max(robot_cov_(0, 0), robot_cov_(1, 1));
    if (max_pos < 1e-9f) return false;
    return (sigma2_theta / max_pos) > params.angular_dominance_ratio;
}

// ===========================================================================
// FIM-based information-gain score
// ===========================================================================
float EpistemicPlanner::score_fim_gain(const Eigen::Vector2f& candidate,
                                      const Eigen::Matrix3f& prior_precision) const
{
    if (room_corners_.empty()) return 0.f;

    auto vis = corner_visibility::visible_corners(candidate, room_corners_,
                                                   params.fim_max_range);
    if (vis.empty()) return 0.f;

    const Eigen::Vector2f dir = candidate - robot_pos();
    const float heading = (dir.squaredNorm() > 1e-4f)
        ? std::atan2(-dir.x(), dir.y())
        : robot_theta();

    const auto fim = corner_visibility::corner_fim(candidate, heading, vis,
                                                    room_corners_, params.fim_corner_sigma);
    return corner_visibility::d_optimality_gain(prior_precision, fim);
}

// ===========================================================================
// evaluate_targets — FIM-based D-optimality scoring + cell score cache
// ===========================================================================
std::vector<EpistemicPlanner::Target> EpistemicPlanner::evaluate_targets() const
{
    // We update mutable cell_scores_ here
    auto& self = const_cast<EpistemicPlanner&>(*this);

    if (!room_bounds_set_ || !robot_state_set_)
    {
        self.cell_scores_.clear();
        return {};
    }

    if (is_angular_dominated())
    {
        Target rot;
        rot.position = robot_pos();
        rot.distance = 0.f;
        rot.rotate_in_place = true;
        rot.score = robot_cov_(2, 2);
        self.cell_scores_.clear();
        return {rot};
    }

    const auto candidates = generate_candidates();
    if (candidates.empty())
    {
        self.cell_scores_.clear();
        return {};
    }

    const Eigen::Matrix3f reg_cov = robot_cov_ + 1e-6f * Eigen::Matrix3f::Identity();
    const Eigen::Matrix3f prior_precision = reg_cov.inverse();
    const auto now = std::chrono::steady_clock::now();

    std::vector<Target> targets;
    targets.reserve(candidates.size());

    for (const auto& pos : candidates)
    {
        Target t;
        t.position = pos;
        t.distance = (pos - robot_pos()).norm();
        const float fim_gain = score_fim_gain(pos, prior_precision);

        // Update cell's running FIM average
        self.visit_grid_.update_fim(pos, fim_gain);

        const float dist_bonus = 1.f + params.w_exploration * t.distance;
        const float ior_bonus = 1.f + params.w_ior * visit_grid_.staleness(pos, params.ior_decay_time, now);
        t.score = fim_gain * dist_bonus * ior_bonus;
        t.eigenvector_score = fim_gain;
        targets.push_back(t);
    }

    std::sort(targets.begin(), targets.end(),
              [](const Target& a, const Target& b) { return a.score > b.score; });

    // Build cell score cache for visualisation (use grid cells, not candidates)
    if (visit_grid_.initialized)
    {
        self.cell_scores_.clear();
        self.cell_scores_.reserve(visit_grid_.cells.size());
        for (int i = 0; i < static_cast<int>(visit_grid_.cells.size()); ++i)
        {
            const auto center = visit_grid_.cell_center(i);
            // Only include cells inside the room polygon
            if (!room_corners_.empty() &&
                !corner_visibility::point_in_polygon(center, room_corners_))
                continue;
            const auto& cell = visit_grid_.cells[i];
            const float stale = visit_grid_.staleness(center, params.ior_decay_time, now);
            const float combined = cell.fim_gain * (1.f + params.w_ior * stale);
            self.cell_scores_.push_back({center, combined});
        }
    }

    return targets;
}

std::optional<EpistemicPlanner::Target> EpistemicPlanner::select_target()
{
    auto targets = evaluate_targets();
    if (targets.empty())
        return std::nullopt;

    // Rotate-in-place: no randomness needed
    if (targets.front().rotate_in_place)
        return targets.front();

    // Weighted random selection from all candidates
    std::vector<float> weights(targets.size());
    for (std::size_t i = 0; i < targets.size(); ++i)
        weights[i] = std::max(0.f, targets[i].score);

    const float total = std::accumulate(weights.begin(), weights.end(), 0.f);
    if (total < 1e-12f)
        return targets.front();

    std::discrete_distribution<std::size_t> dist(weights.begin(), weights.end());
    return targets[dist(rng_)];
}

// ===========================================================================
// update_target — handles dwell, arrival, and fresh selection
// ===========================================================================
std::optional<EpistemicPlanner::Target> EpistemicPlanner::update_target()
{
    // ---- Dwell check ----
    if (dwelling_)
    {
        if (std::chrono::steady_clock::now() < dwell_until_)
            return current_target_;   // still dwelling → keep current target
        dwelling_ = false;
        current_target_.reset();
    }

    // ---- Update visit grid ----
    visit_grid_.mark_visited(robot_pos());

    // ---- Arrival check ----
    if (current_target_.has_value() && !current_target_->rotate_in_place)
    {
        const float dist = (current_target_->position - robot_pos()).norm();
        if (dist < params.arrival_distance)
        {
            dwelling_ = true;
            dwell_until_ = std::chrono::steady_clock::now()
                         + std::chrono::milliseconds(static_cast<int>(params.dwell_time * 1000.f));
            return current_target_;
        }
    }

    // ---- Select new target if needed ----
    if (!current_target_.has_value())
    {
        auto target_opt = select_target();
        if (!target_opt)
            return std::nullopt;
        current_target_ = *target_opt;
    }

    return current_target_;
}

} // namespace rc
