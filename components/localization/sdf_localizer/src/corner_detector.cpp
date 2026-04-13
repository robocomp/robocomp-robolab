#include "corner_detector.h"
#include <cmath>
#include <QDebug>

namespace rc {

// ---------------------------------------------------------------------------
//  set_model_corners — keep only real corners and store wall edge directions
// ---------------------------------------------------------------------------
void CornerDetector::set_model_corners(const std::vector<Eigen::Vector2f>& polygon_vertices)
{
    model_corners_.clear();
    const int N = static_cast<int>(polygon_vertices.size());
    if (N < 3) return;

    for (int i = 0; i < N; ++i)
    {
        const Eigen::Vector2f& prev = polygon_vertices[(i + N - 1) % N];
        const Eigen::Vector2f& curr = polygon_vertices[i];
        const Eigen::Vector2f& next = polygon_vertices[(i + 1) % N];

        const Eigen::Vector2f d1 = (prev - curr).normalized();
        const Eigen::Vector2f d2 = (next - curr).normalized();
        const float dot = std::clamp(d1.dot(d2), -1.0f, 1.0f);
        const float angle_deg = std::acos(dot) * 180.0f / static_cast<float>(M_PI);

        if (angle_deg >= params_.min_corner_angle && angle_deg <= params_.max_corner_angle)
        {
            ModelCorner mc;
            mc.position       = curr;
            mc.edge_in_dir    = (curr - prev).normalized();  // wall arriving at corner
            mc.edge_out_dir   = (next - curr).normalized();  // wall leaving corner
            mc.original_index = i;
            model_corners_.push_back(mc);
        }
    }
    qInfo() << "CornerDetector: kept" << model_corners_.size() << "of"
            << N << "polygon vertices as real corners";
}

// ---------------------------------------------------------------------------
//  detect — model-guided partitioning + PCA line fit + intersect
// ---------------------------------------------------------------------------
CornerDetector::DetectionResult CornerDetector::detect(
        const std::vector<Eigen::Vector3f>& lidar_points,
        float robot_x, float robot_y, float robot_theta,
        float max_range) const
{
    DetectionResult result;
    if (model_corners_.empty() || lidar_points.empty())
        return result;

    const bool verbose = false;

    // Build 2D lidar points in robot frame (drop z)
    std::vector<Eigen::Vector2f> pts2d;
    pts2d.reserve(lidar_points.size());
    for (const auto& p : lidar_points)
        pts2d.emplace_back(p.x(), p.y());

    // Rotation world→robot:  R(-θ)
    const float cos_t = std::cos(robot_theta);
    const float sin_t = std::sin(robot_theta);
    const Eigen::Vector2f t_world(robot_x, robot_y);

    // Helper: rotate a world-frame vector into robot frame
    auto to_robot = [&](const Eigen::Vector2f& v) -> Eigen::Vector2f {
        return {cos_t * v.x() + sin_t * v.y(),
               -sin_t * v.x() + cos_t * v.y()};
    };

    const float search_r2 = params_.search_radius * params_.search_radius;
    const float max_range2 = max_range * max_range;

    if (verbose)
        qInfo() << "[CD] pts2d=" << pts2d.size() << "model=" << model_corners_.size()
                << "pose=(" << robot_x << robot_y << robot_theta << ")"
                << "search_r=" << params_.search_radius;

    for (const auto& mc : model_corners_)
    {
        // Project model corner into robot frame
        const Eigen::Vector2f dw = mc.position - t_world;
        if (dw.squaredNorm() > max_range2)
        {
            if (verbose) qInfo() << "  corner" << mc.original_index << "too far:" << dw.norm() << "m";
            continue;
        }

        const Eigen::Vector2f predicted = to_robot(dw);
        result.corners_in_fov++;

        // Project the two wall directions into robot frame
        const Eigen::Vector2f dir_in  = to_robot(mc.edge_in_dir);
        const Eigen::Vector2f dir_out = to_robot(mc.edge_out_dir);

        // Wall normals (perpendicular to each edge direction)
        const Eigen::Vector2f normal_in (-dir_in.y(),  dir_in.x());
        const Eigen::Vector2f normal_out(-dir_out.y(), dir_out.x());

        // Gather neighbourhood and partition by closest wall
        std::vector<Eigen::Vector2f> group_in, group_out;
        group_in.reserve(128);
        group_out.reserve(128);

        for (const auto& p : pts2d)
        {
            const Eigen::Vector2f d = p - predicted;
            if (d.squaredNorm() > search_r2)
                continue;
            // Distance to each wall line (through the predicted corner)
            const float dist_to_in  = std::abs(normal_in.dot(d));
            const float dist_to_out = std::abs(normal_out.dot(d));
            if (dist_to_in < dist_to_out)
                group_in.push_back(p);
            else
                group_out.push_back(p);
        }

        if (verbose)
            qInfo() << "  corner" << mc.original_index
                    << "world=(" << mc.position.x() << mc.position.y()
                    << ") pred=(" << predicted.x() << predicted.y()
                    << ") in=" << group_in.size() << "out=" << group_out.size();

        if (static_cast<int>(group_in.size()) < params_.min_points_per_line ||
            static_cast<int>(group_out.size()) < params_.min_points_per_line)
        {
            if (verbose) qInfo() << "    not enough points in one group";
            continue;
        }

        // --- PCA line fit on each group ---
        auto line_in  = fit_line_pca(group_in,  params_.min_points_per_line);
        auto line_out = fit_line_pca(group_out, params_.min_points_per_line);
        if (!line_in || !line_out)
        {
            if (verbose) qInfo() << "    PCA fit failed";
            continue;
        }

        result.corners_detected++;

        // --- Intersect ---
        float angle_deg = 0.f;
        auto intersection = intersect(*line_in, *line_out, &angle_deg);
        if (!intersection)
        {
            if (verbose) qInfo() << "    intersect failed (parallel)";
            continue;
        }

        // --- Quality filters ---
        if (angle_deg < params_.min_corner_angle || angle_deg > params_.max_corner_angle)
        {
            if (verbose) qInfo() << "    angle rejected:" << angle_deg;
            continue;
        }

        const float dist = (*intersection - predicted).norm();
        if (dist > params_.max_match_distance)
        {
            if (verbose) qInfo() << "    distance rejected:" << dist;
            continue;
        }

        // --- Orientation consistency: PCA line direction must align with model edge ---
        {
            const float cos_thresh = std::cos(params_.max_orientation_dev * static_cast<float>(M_PI) / 180.f);
            // line direction() is ⊥ to normal; model dirs are in robot frame
            const float dot_in  = std::abs(line_in->direction().dot(dir_in));
            const float dot_out = std::abs(line_out->direction().dot(dir_out));
            if (dot_in < cos_thresh || dot_out < cos_thresh)
            {
                if (verbose) qInfo() << "    orientation rejected: dot_in=" << dot_in << "dot_out=" << dot_out;
                continue;
            }
        }

        result.corners_accepted++;

        // Covariance: σ² (n_in n_in^T + n_out n_out^T)
        const float sigma = params_.ransac_threshold;
        Eigen::Matrix2f cov = sigma * sigma *
            (line_in->normal * line_in->normal.transpose() +
             line_out->normal * line_out->normal.transpose());

        CornerMatch match;
        match.model_index = mc.original_index;
        match.detected    = *intersection;
        match.predicted   = predicted;
        match.model_world = mc.position;
        match.distance    = dist;
        match.angle_deg   = angle_deg;
        match.covariance  = cov;
        result.matches.push_back(match);
    }

    return result;
}

// ---------------------------------------------------------------------------
//  fit_line_pca — least-squares line fit via PCA
// ---------------------------------------------------------------------------
std::optional<CornerDetector::Line2D> CornerDetector::fit_line_pca(
        const std::vector<Eigen::Vector2f>& pts, int min_points)
{
    if (static_cast<int>(pts.size()) < min_points)
        return std::nullopt;

    Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
    for (const auto& p : pts)
        centroid += p;
    centroid /= static_cast<float>(pts.size());

    Eigen::Matrix2f scatter = Eigen::Matrix2f::Zero();
    for (const auto& p : pts)
    {
        Eigen::Vector2f dp = p - centroid;
        scatter += dp * dp.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(scatter);
    Line2D line;
    line.normal = eig.eigenvectors().col(0);  // smallest eigenvalue = line normal
    line.d = line.normal.dot(centroid);
    return line;
}

// ---------------------------------------------------------------------------
//  intersect
// ---------------------------------------------------------------------------
std::optional<Eigen::Vector2f> CornerDetector::intersect(
        const Line2D& a, const Line2D& b, float* angle_deg)
{
    const float det = a.normal.x() * b.normal.y() - a.normal.y() * b.normal.x();
    if (std::abs(det) < 1e-6f)
        return std::nullopt;

    const float x = (b.normal.y() * a.d - a.normal.y() * b.d) / det;
    const float y = (a.normal.x() * b.d - b.normal.x() * a.d) / det;

    if (angle_deg)
    {
        // Angle between the two wall directions (= 180° - angle between normals)
        const float ndot = std::abs(a.normal.dot(b.normal));
        const float clamped = std::min(1.0f, ndot);
        *angle_deg = 180.0f - std::acos(clamped) * 180.0f / static_cast<float>(M_PI);
    }

    return Eigen::Vector2f(x, y);
}

} // namespace rc
