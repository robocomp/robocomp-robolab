#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Dense>

namespace rc {
namespace corner_visibility {

/// Point-in-polygon test (ray-casting algorithm).
/// Polygon is open: N vertices, implicit edge from vertex[N-1] to vertex[0].
inline bool point_in_polygon(const Eigen::Vector2f& pt,
                             const std::vector<Eigen::Vector2f>& poly)
{
    const int n = static_cast<int>(poly.size());
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++)
    {
        if (((poly[i].y() > pt.y()) != (poly[j].y() > pt.y())) &&
            (pt.x() < (poly[j].x() - poly[i].x()) * (pt.y() - poly[i].y())
                        / (poly[j].y() - poly[i].y()) + poly[i].x()))
            inside = !inside;
    }
    return inside;
}

/// Test whether two 2D segments (p1→p2) and (p3→p4) properly cross.
/// Returns false for collinear or endpoint-only contacts.
inline bool segments_cross(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2,
                           const Eigen::Vector2f& p3, const Eigen::Vector2f& p4)
{
    const Eigen::Vector2f d1 = p2 - p1;
    const Eigen::Vector2f d2 = p4 - p3;
    const float denom = d1.x() * d2.y() - d1.y() * d2.x();
    if (std::abs(denom) < 1e-10f) return false;

    const Eigen::Vector2f d3 = p3 - p1;
    const float t = (d3.x() * d2.y() - d3.y() * d2.x()) / denom;
    const float u = (d3.x() * d1.y() - d3.y() * d1.x()) / denom;
    constexpr float eps = 1e-4f;
    return t > eps && t < (1.f - eps) && u > eps && u < (1.f - eps);
}

/// Check whether polygon corner[corner_idx] is visible from robot_pos.
/// A corner is visible when the sight-line does not cross any non-adjacent
/// wall segment and the corner is within max_range.
inline bool is_corner_visible(const Eigen::Vector2f& robot_pos,
                              int corner_idx,
                              const std::vector<Eigen::Vector2f>& polygon,
                              float max_range = 15.f)
{
    const int N = static_cast<int>(polygon.size());
    if (corner_idx < 0 || corner_idx >= N) return false;

    const Eigen::Vector2f& corner = polygon[corner_idx];
    if ((corner - robot_pos).squaredNorm() > max_range * max_range)
        return false;

    // Adjacent wall segments touch the corner → skip them
    const int seg_before = (corner_idx - 1 + N) % N; // edge seg_before → corner_idx
    const int seg_after  = corner_idx;                 // edge corner_idx → (corner_idx+1)%N

    for (int i = 0; i < N; ++i)
    {
        if (i == seg_before || i == seg_after) continue;
        if (segments_cross(robot_pos, corner, polygon[i], polygon[(i + 1) % N]))
            return false;
    }
    return true;
}

/// Return indices of all polygon corners visible from robot_pos.
inline std::vector<int> visible_corners(const Eigen::Vector2f& robot_pos,
                                        const std::vector<Eigen::Vector2f>& polygon,
                                        float max_range = 15.f)
{
    std::vector<int> result;
    for (int i = 0; i < static_cast<int>(polygon.size()); ++i)
        if (is_corner_visible(robot_pos, i, polygon, max_range))
            result.push_back(i);
    return result;
}

/// 2×3 observation Jacobian for one corner.
///
/// Observation model:  z = R(θ)ᵀ · (c_world − [x,y])
///
///   J = ∂z/∂(x,y,θ) = [[-cosθ, -sinθ,   z₁],
///                       [ sinθ, -cosθ,  -z₀]]
///
/// where z₀ = cosθ·Δx + sinθ·Δy , z₁ = −sinθ·Δx + cosθ·Δy .
inline Eigen::Matrix<float, 2, 3> corner_observation_jacobian(
    const Eigen::Vector2f& robot_pos, float robot_theta,
    const Eigen::Vector2f& corner_world)
{
    const float ct = std::cos(robot_theta);
    const float st = std::sin(robot_theta);
    const float dx = corner_world.x() - robot_pos.x();
    const float dy = corner_world.y() - robot_pos.y();
    const float z0 =  ct * dx + st * dy;
    const float z1 = -st * dx + ct * dy;

    Eigen::Matrix<float, 2, 3> J;
    J << -ct, -st,  z1,
          st, -ct, -z0;
    return J;
}

/// Summed 3×3 Fisher Information Matrix from observing a set of visible
/// corners, assuming isotropic detection noise R = σ²·I₂ per corner.
///
///   I_total = Σ_j (1/σ²) Jⱼᵀ Jⱼ
inline Eigen::Matrix3f corner_fim(const Eigen::Vector2f& robot_pos,
                                   float robot_theta,
                                   const std::vector<int>& vis_indices,
                                   const std::vector<Eigen::Vector2f>& polygon,
                                   float corner_sigma = 0.04f)
{
    Eigen::Matrix3f fim = Eigen::Matrix3f::Zero();
    const float inv_var = 1.f / (corner_sigma * corner_sigma);
    for (int idx : vis_indices)
    {
        const auto J = corner_observation_jacobian(robot_pos, robot_theta, polygon[idx]);
        fim.noalias() += inv_var * (J.transpose() * J);
    }
    return fim;
}

/// D-optimality information gain:
///   gain = log det(Λ_prior + I_corners) − log det(Λ_prior)
/// Higher ⟹ more informative.  Returns 0 when no corners contribute.
inline float d_optimality_gain(const Eigen::Matrix3f& prior_precision,
                               const Eigen::Matrix3f& corner_fim_mat)
{
    const float ld_prior = std::log(std::max(prior_precision.determinant(), 1e-30f));
    const float ld_post  = std::log(std::max((prior_precision + corner_fim_mat).determinant(), 1e-30f));
    return ld_post - ld_prior;
}

} // namespace corner_visibility
} // namespace rc
