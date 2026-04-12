#pragma once

#include <vector>
#include <optional>
#include <Eigen/Dense>

namespace rc {

/// Detects room corners in lidar scans using model-guided partitioning.
///
/// Pipeline:
///   1. Project model corners (world frame) into robot frame.
///   2. For each predicted corner, gather lidar points within a search radius.
///   3. Partition the neighbourhood into two groups using the known wall
///      directions from the polygon model.
///   4. Fit a line (PCA) to each group and intersect.
///   5. Accept the detection if it passes angle and distance quality gates.
class CornerDetector
{
public:
    // ===== Configuration =====
    struct Params
    {
        float search_radius       = 1.5f;   // meters around predicted corner to gather points
        int   min_points_per_line = 3;       // minimum points per wall group
        float ransac_threshold    = 0.06f;   // inlier band width for optional outlier rejection
        float max_match_distance  = 1.5f;   // max distance between detected and predicted corner
        float min_corner_angle    = 25.0f;   // degrees — reject nearly-parallel intersections
        float max_corner_angle    = 155.0f;  // degrees — reject nearly-flat intersections
    };

    // ===== Output types =====

    struct CornerMatch
    {
        int    model_index;         // index into the ORIGINAL polygon vertex list
        Eigen::Vector2f detected;   // detected position (robot frame, meters)
        Eigen::Vector2f predicted;  // predicted position (robot frame, meters)
        float  distance;            // ||detected - predicted||
        float  angle_deg;           // angle between the two fitted lines
        Eigen::Matrix2f covariance; // 2×2 detection uncertainty (robot frame)
    };

    struct DetectionResult
    {
        std::vector<CornerMatch> matches;
        int corners_in_fov = 0;
        int corners_detected = 0;
        int corners_accepted = 0;
    };

    // ===== Interface =====

    explicit CornerDetector() = default;
    explicit CornerDetector(const Params& p) : params_(p) {}

    void set_model_corners(const std::vector<Eigen::Vector2f>& polygon_vertices);

    DetectionResult detect(const std::vector<Eigen::Vector3f>& lidar_points,
                           float robot_x, float robot_y, float robot_theta,
                           float max_range = 15.0f) const;

    Params& params() { return params_; }
    const Params& params() const { return params_; }

private:
    Params params_;

    /// Model corner with its two adjacent wall directions.
    struct ModelCorner
    {
        Eigen::Vector2f position;       // world frame
        Eigen::Vector2f edge_in_dir;    // unit direction of wall arriving at this corner
        Eigen::Vector2f edge_out_dir;   // unit direction of wall leaving this corner
        int original_index;             // index in original polygon
    };
    std::vector<ModelCorner> model_corners_;

    /// 2D line: normal · p = d   (normal is unit length)
    struct Line2D
    {
        Eigen::Vector2f normal;
        float d;
        Eigen::Vector2f direction() const { return Eigen::Vector2f(-normal.y(), normal.x()); }
    };

    /// PCA line fit — returns nullopt if fewer than min_points.
    static std::optional<Line2D> fit_line_pca(const std::vector<Eigen::Vector2f>& pts,
                                               int min_points);

    /// Intersect two lines.  Returns nullopt if (nearly) parallel.
    static std::optional<Eigen::Vector2f> intersect(const Line2D& a, const Line2D& b,
                                                     float* angle_deg = nullptr);
};

} // namespace rc
