//
// Created by pbustos on 21/11/25.
//

#ifndef ROBUST_ROOM_CENTER_ESTIMATOR_H
#define ROBUST_ROOM_CENTER_ESTIMATOR_H

#include <Eigen/Dense>
#include <vector>
#include <optional>
#include <Lidar3D.h>

namespace rc
{
    class PointcloudCenterEstimator
    {
    public:
        struct Config
        {
            int num_sectors = 360;              // 10° sectors
            double max_range = 30000.0;           // meters
            double min_range = 200;           // Remove robot hardware
            double outlier_std_threshold = 2500; // Statistical outlier removal
            double obb_fit_tolerance = 100;    // Bounding box fit tolerance
            size_t min_valid_points = 20;      // Minimum points required

            Config(){}
        };

        using Point2D = Eigen::Vector2d;

        struct OBB
        {
            Point2D center{};
            double width = 0.0;
            double height = 0.0;
            double rotation = 0.0; // radians
        };

        explicit PointcloudCenterEstimator(const Config &config = Config{});

        std::optional<Point2D> estimate(const std::vector<Point2D>& points);
        std::optional<Point2D> estimate(const std::vector<Eigen::Vector3f>& points);

        std::optional<OBB> estimate_obb(const std::vector<Point2D>& points);
        std::optional<OBB> estimate_obb(const std::vector<Eigen::Vector3f>& points);

    private:
        Config config_;

        std::vector<Point2D> filterPoints(const std::vector<Point2D>& points);
        std::vector<Point2D> extractBoundaryPoints(const std::vector<Point2D>& points);
        bool isLocalMaximum(const Point2D& candidate,
                           const std::vector<Point2D>& neighbors,
                           double threshold);
        std::vector<Point2D> removeStatisticalOutliers(const std::vector<Point2D>& points);
        Point2D calculateRobustCentroid(const std::vector<Point2D>& points);
        std::vector<Point2D> computeConvexHull(const std::vector<Point2D>& points);


        OBB computeOBB(const std::vector<Point2D>& hull);
    };


};
#endif // ROBUST_ROOM_CENTER_ESTIMATOR_H