#include "svg_room_loader.h"

#include <QDebug>
#include <QDomDocument>
#include <QFile>
#include <QFileInfo>
#include <QStringList>

namespace rc
{

std::vector<Eigen::Vector2f> SvgRoomLoader::load_polygon_points(const std::string& svg_file,
                                                                const std::string& polygon_id,
                                                                bool flip_y,
                                                                bool mirror_x)
{
    const QString svg_path = QString::fromStdString(svg_file);
    QFile file(svg_path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qWarning() << "SvgRoomLoader: cannot open SVG file:" << svg_path;
        return {};
    }

    QDomDocument doc;
    QString parse_error;
    int error_line = 0;
    int error_col = 0;
    if (!doc.setContent(&file, &parse_error, &error_line, &error_col))
    {
        qWarning() << "SvgRoomLoader: XML parse error in" << svg_path
                   << "line" << error_line << "col" << error_col << ":" << parse_error;
        return {};
    }

    const QString target_id = QString::fromStdString(polygon_id);
    const QDomNodeList polygons = doc.elementsByTagName("polygon");
    for (int i = 0; i < polygons.count(); ++i)
    {
        const QDomElement poly = polygons.at(i).toElement();
        if (poly.isNull())
            continue;

        if (poly.attribute("id") != target_id)
            continue;

        const QString points_attr = poly.attribute("points");
        auto points = parse_points_attribute(points_attr, flip_y, mirror_x);
        if (points.size() < 3)
        {
            qWarning() << "SvgRoomLoader: polygon" << target_id
                       << "in" << svg_path << "has fewer than 3 points.";
            return {};
        }

        // Reflection flips winding. Reverse to keep original CW/CCW orientation.
        if (mirror_x)
            std::reverse(points.begin(), points.end());

        return points;
    }

    qWarning() << "SvgRoomLoader: polygon id not found:" << target_id << "in" << svg_path;
    return {};
}

std::vector<Eigen::Vector2f> SvgRoomLoader::parse_points_attribute(const QString& points_attr,
                                                                   bool flip_y,
                                                                   bool mirror_x)
{
    std::vector<Eigen::Vector2f> points;
    const QStringList tokens = points_attr.simplified().split(' ', Qt::SkipEmptyParts);

    points.reserve(tokens.size());
    for (const QString& token : tokens)
    {
        const QStringList xy = token.split(',', Qt::SkipEmptyParts);
        if (xy.size() != 2)
            continue;

        bool ok_x = false;
        bool ok_y = false;
        const float x = xy[0].toFloat(&ok_x);
        float y = xy[1].toFloat(&ok_y);
        if (!ok_x || !ok_y)
            continue;

        float x_final = x;
        if (mirror_x)
            x_final = -x_final;

        if (flip_y)
            y = -y;

        points.emplace_back(x_final, y);
    }

    return points;
}

} // namespace rc
