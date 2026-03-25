#pragma once

#include <Eigen/Dense>
#include <QString>

#include <string>
#include <vector>

namespace rc
{
class SvgRoomLoader
{
public:
    // Reads a polygon from an SVG file ("polygon" element with the given id) and returns 2D points.
    static std::vector<Eigen::Vector2f> load_polygon_points(const std::string& svg_file,
                                                             const std::string& polygon_id = "room_contour",
                                                             bool flip_y = false,
                                                             bool mirror_x = false);

private:
    static std::vector<Eigen::Vector2f> parse_points_attribute(const QString& points_attr,
                                                               bool flip_y,
                                                               bool mirror_x);
};

} // namespace rc
