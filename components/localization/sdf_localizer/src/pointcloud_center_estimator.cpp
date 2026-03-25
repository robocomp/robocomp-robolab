//
// Created by pbustos on 21/11/25.
//

#include "pointcloud_center_estimator.h"
#include <cmath>
#include <algorithm>
#include <numbers>
#include <random>
#include <numeric>
#include <ATen/core/interned_strings.h>

namespace rc
{
    PointcloudCenterEstimator::PointcloudCenterEstimator(const Config &config)
        : config_(config) {}

    std::optional<PointcloudCenterEstimator::Point2D>
    PointcloudCenterEstimator::estimate(const std::vector<Point2D>& points) {
        const auto obb = estimate_obb(points);
        if(!obb.has_value()) return std::nullopt;
        return obb->center;
    }

    std::optional<PointcloudCenterEstimator::Point2D> PointcloudCenterEstimator::estimate( const std::vector<Eigen::Vector3f> &points)
    {
        std::vector<Point2D> pc;
        pc.reserve(points.size());
        for (const auto &p : points)
            pc.emplace_back(p.head(2).cast<double>());
        return estimate(pc);
    }

    std::optional<PointcloudCenterEstimator::OBB>
    PointcloudCenterEstimator::estimate_obb(const std::vector<Point2D>& points)
    {
        if (points.size() < config_.min_valid_points)
            return std::nullopt;

        auto cleaned = filterPoints(points);
        if (cleaned.size() < 8)
        {
            // fallback: no good boundary -> centroid, no rotation
            OBB obb;
            obb.center = calculateRobustCentroid(points);
            return obb;
        }

        auto boundary = extractBoundaryPoints(cleaned);
        if (boundary.size() < 4)
        {
            OBB obb;
            obb.center = calculateRobustCentroid(cleaned);
            return obb;
        }

        boundary = removeStatisticalOutliers(boundary);

        auto hull = computeConvexHull(boundary);
        if (hull.size() < 3)
        {
            OBB obb;
            obb.center = calculateRobustCentroid(cleaned);
            return obb;
        }

        return computeOBB(hull);
    }

    std::optional<PointcloudCenterEstimator::OBB>
    PointcloudCenterEstimator::estimate_obb(const std::vector<Eigen::Vector3f>& points)
    {
        std::vector<Point2D> pc;
        pc.reserve(points.size());
        for (const auto &p : points)
            pc.emplace_back(p.head(2).cast<double>());
        return estimate_obb(pc);
    }

    std::vector<PointcloudCenterEstimator::Point2D>
    PointcloudCenterEstimator::filterPoints(const std::vector<Point2D>& points) {
        std::vector<Point2D> filtered;
        filtered.reserve(points.size());

        for (const auto& p : points) {
            double r = p.norm();
            if (r >= config_.min_range && r <= config_.max_range) {
                filtered.push_back(p);
            }
        }

        return filtered;
    }

    std::vector<PointcloudCenterEstimator::Point2D>
    PointcloudCenterEstimator::extractBoundaryPoints(const std::vector<Point2D>& points) {
        std::vector<Point2D> boundary;
        boundary.reserve(config_.num_sectors);

        const double sector_angle = 2.0 * std::numbers::pi / config_.num_sectors;
        std::vector<std::vector<Point2D>> sectors(config_.num_sectors);

        for (const auto& p : points) {
            double angle = std::atan2(p.y(), p.x());
            if (angle < 0) angle += 2.0 * std::numbers::pi;

            int sector_idx = static_cast<int>(angle / sector_angle);
            if (sector_idx == config_.num_sectors) sector_idx = 0;

            sectors[sector_idx].push_back(p);
        }

        for (const auto& sector : sectors) {
            if (sector.empty()) continue;

            auto farthest = std::max_element(sector.begin(), sector.end(),
                [](const Point2D& a, const Point2D& b) {
                    return a.norm() < b.norm();
                });

            if (isLocalMaximum(*farthest, sector, 0.5)) {
                boundary.push_back(*farthest);
            }
        }

        return boundary;
    }

    bool PointcloudCenterEstimator::isLocalMaximum(
        const Point2D& candidate,
        const std::vector<Point2D>& neighbors,
        double threshold) {
        double candidate_range = candidate.norm();

        return std::all_of(neighbors.begin(), neighbors.end(),
            [&](const Point2D& p) {
                return p.norm() <= candidate_range + threshold;
            });
    }

    std::vector<PointcloudCenterEstimator::Point2D>
    PointcloudCenterEstimator::removeStatisticalOutliers(const std::vector<Point2D>& points) {
        if (points.size() < 6) return points;

        const int k = 5;
        std::vector<double> avg_distances;
        avg_distances.reserve(points.size());

        for (size_t i = 0; i < points.size(); ++i) {
            std::vector<double> distances;
            distances.reserve(points.size());

            for (size_t j = 0; j < points.size(); ++j) {
                if (i != j) {
                    distances.push_back((points[i] - points[j]).norm());
                }
            }

            std::nth_element(distances.begin(), distances.begin() + k, distances.end());
            double avg_k_dist = std::accumulate(distances.begin(), distances.begin() + k, 0.0) / k;
            avg_distances.push_back(avg_k_dist);
        }

        double mean = std::accumulate(avg_distances.begin(), avg_distances.end(), 0.0) / avg_distances.size();
        double sq_sum = std::inner_product(avg_distances.begin(), avg_distances.end(),
                                          avg_distances.begin(), 0.0);
        double stdev = std::sqrt(sq_sum / avg_distances.size() - mean * mean);

        std::vector<Point2D> filtered;
        filtered.reserve(points.size());

        for (size_t i = 0; i < points.size(); ++i) {
            if (std::abs(avg_distances[i] - mean) < config_.outlier_std_threshold * stdev) {
                filtered.push_back(points[i]);
            }
        }

        return filtered;
    }

    PointcloudCenterEstimator::Point2D
    PointcloudCenterEstimator::calculateRobustCentroid(const std::vector<Point2D>& points) {
        std::vector<double> xs, ys;
        xs.reserve(points.size());
        ys.reserve(points.size());

        for (const auto& p : points) {
            xs.push_back(p.x());
            ys.push_back(p.y());
        }

        auto median = [](std::vector<double>& v) {
            auto mid = v.begin() + v.size() / 2;
            std::nth_element(v.begin(), mid, v.end());
            return *mid;
        };

        return {median(xs), median(ys)};
    }

    std::vector<PointcloudCenterEstimator::Point2D>
    PointcloudCenterEstimator::computeConvexHull(const std::vector<Point2D>& points) {
        if (points.size() <= 3) return points;

        size_t pivot_idx = 0;
        for (size_t i = 1; i < points.size(); ++i) {
            if (points[i].y() < points[pivot_idx].y() ||
                (points[i].y() == points[pivot_idx].y() && points[i].x() < points[pivot_idx].x())) {
                pivot_idx = i;
            }
        }

        Point2D pivot = points[pivot_idx];
        std::vector<Point2D> sorted = points;

        std::sort(sorted.begin(), sorted.end(),
            [&pivot](const Point2D& a, const Point2D& b) {
                double angle_a = std::atan2(a.y() - pivot.y(), a.x() - pivot.x());
                double angle_b = std::atan2(b.y() - pivot.y(), b.x() - pivot.x());
                return angle_a < angle_b;
            });

        std::vector<Point2D> hull;
        hull.push_back(sorted[0]);
        hull.push_back(sorted[1]);

        for (size_t i = 2; i < sorted.size(); ++i) {
            while (hull.size() >= 2) {
                const Point2D& b = hull.back();
                const Point2D& a = hull[hull.size() - 2];
                Point2D ab = b - a;
                Point2D ac = sorted[i] - a;

                double cross = ab.x() * ac.y() - ab.y() * ac.x();

                if (cross > 0) break;
                hull.pop_back();
            }
            hull.push_back(sorted[i]);
        }

        return hull;
    }

    PointcloudCenterEstimator::OBB
    PointcloudCenterEstimator::computeOBB(const std::vector<Point2D>& hull) {
        if (hull.size() < 3) {
            auto [min_x, max_x] = std::minmax_element(hull.begin(), hull.end(),
                [](const Point2D& a, const Point2D& b) { return a.x() < b.x(); });
            auto [min_y, max_y] = std::minmax_element(hull.begin(), hull.end(),
                [](const Point2D& a, const Point2D& b) { return a.y() < b.y(); });

            return {
                {(min_x->x() + max_x->x()) / 2.0, (min_y->y() + max_y->y()) / 2.0},
                max_x->x() - min_x->x(),
                max_y->y() - min_y->y(),
                0.0
            };
        }

        double min_area = std::numeric_limits<double>::max();
        OBB best_obb;

        for (size_t i = 0; i < hull.size(); ++i) {
            const Point2D& p1 = hull[i];
            const Point2D& p2 = hull[(i + 1) % hull.size()];

            double angle = std::atan2(p2.y() - p1.y(), p2.x() - p1.x());
            Eigen::Rotation2Dd rot(-angle);

            std::vector<Point2D> rotated;
            rotated.reserve(hull.size());

            for (const auto& p : hull)
                rotated.push_back(rot * p);

            auto [min_x, max_x] = std::minmax_element(rotated.begin(), rotated.end(),
                [](const Point2D& a, const Point2D& b) { return a.x() < b.x(); });
            auto [min_y, max_y] = std::minmax_element(rotated.begin(), rotated.end(),
                [](const Point2D& a, const Point2D& b) { return a.y() < b.y(); });

            double width = max_x->x() - min_x->x();
            double height = max_y->y() - min_y->y();
            double area = width * height;

            if (area < min_area) {
                min_area = area;

                Point2D center_rot((min_x->x() + max_x->x()) / 2.0,
                                  (min_y->y() + max_y->y()) / 2.0);

                best_obb.center = rot.inverse() * center_rot;
                best_obb.width = width;
                best_obb.height = height;
                best_obb.rotation = angle;
            }
        }
        return best_obb;
    }
}
