#include "viewer_2d.h"

#include <abstract_graphic_viewer/abstract_graphic_viewer.h>

#include <QPen>
#include <QBrush>
#include <QPolygonF>
#include <QtMath>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace rc {

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────
Viewer2D::Viewer2D(QWidget* parent, const QRectF& grid_dim, bool show_axis)
{
    agv_ = new AbstractGraphicViewer(parent, grid_dim, show_axis);

    // Forward all AGV signals as Viewer2D signals
    connect(agv_, &AbstractGraphicViewer::robot_moved,
            this, &Viewer2D::robot_moved);
    connect(agv_, &AbstractGraphicViewer::robot_rotate,
            this, &Viewer2D::robot_rotate);
    connect(agv_, &AbstractGraphicViewer::robot_dragging,
            this, &Viewer2D::robot_dragging);
    connect(agv_, &AbstractGraphicViewer::robot_drag_end,
            this, &Viewer2D::robot_drag_end);
    connect(agv_, &AbstractGraphicViewer::new_mouse_coordinates,
            this, &Viewer2D::new_mouse_coordinates);
    connect(agv_, &AbstractGraphicViewer::right_click,
            this, &Viewer2D::right_click);
}

// ─────────────────────────────────────────────────────────────────────────────
// Widget / view management
// ─────────────────────────────────────────────────────────────────────────────
QWidget* Viewer2D::get_widget() const { return agv_; }

void Viewer2D::add_robot(float w, float l, float offset, float rotation, QColor color)
{
    agv_->add_robot(w, l, offset, rotation, color);
}

void Viewer2D::show() { agv_->show(); }

QTransform Viewer2D::transform() const { return agv_->transform(); }

void Viewer2D::set_transform(const QTransform& t) { agv_->setTransform(t); }

void Viewer2D::fit_to_scene(const QRectF& r) { agv_->fitToScene(r); }

void Viewer2D::fit_view(float margin_ratio)
{
    QRectF bounds;
    if (polygon_item_ != nullptr)
        bounds = polygon_item_->boundingRect().translated(polygon_item_->pos());
    else if (estimated_room_item_ != nullptr)
        bounds = estimated_room_item_->boundingRect().translated(estimated_room_item_->pos());
    else
        bounds = agv_->scene.itemsBoundingRect();

    if (!bounds.isValid() || bounds.isEmpty())
        return;

    const qreal dx = std::max(0.1, bounds.width() * std::max(0.f, margin_ratio));
    const qreal dy = std::max(0.1, bounds.height() * std::max(0.f, margin_ratio));
    agv_->fitToScene(bounds.adjusted(-dx, -dy, dx, dy));
}

void Viewer2D::fit_to_robot_and_points(const Eigen::Affine2f& robot_pose,
                                       const std::vector<Eigen::Vector3f>& lidar_points,
                                       float fit_radius,
                                       float margin_ratio)
{
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();

    const Eigen::Vector2f robot_center = robot_pose.translation();
    min_x = std::min(min_x, robot_center.x());
    max_x = std::max(max_x, robot_center.x());
    min_y = std::min(min_y, robot_center.y());
    max_y = std::max(max_y, robot_center.y());

    const float clamped_radius = std::max(1.0f, fit_radius);
    const float fit_radius_sq = clamped_radius * clamped_radius;
    for (const auto& p : lidar_points)
    {
        const Eigen::Vector2f pr = p.head<2>();
        const Eigen::Vector2f pw = robot_pose.linear() * pr + robot_center;
        const Eigen::Vector2f d = pw - robot_center;
        if (d.squaredNorm() > fit_radius_sq)
            continue;

        min_x = std::min(min_x, pw.x());
        max_x = std::max(max_x, pw.x());
        min_y = std::min(min_y, pw.y());
        max_y = std::max(max_y, pw.y());
    }

    const float width = std::max(2.0f, max_x - min_x);
    const float height = std::max(2.0f, max_y - min_y);
    const float mr = std::max(0.f, margin_ratio);
    const float margin_x = std::max(0.3f, width * mr);
    const float margin_y = std::max(0.3f, height * mr);

    fit_to_scene(QRectF(min_x - margin_x,
                        min_y - margin_y,
                        width + 2.f * margin_x,
                        height + 2.f * margin_y));
}

void Viewer2D::center_on(float x, float y) { agv_->centerOn(x, y); }

// ─────────────────────────────────────────────────────────────────────────────
// Robot
// ─────────────────────────────────────────────────────────────────────────────
void Viewer2D::update_robot(const Eigen::Affine2f& robot_pose)
{
    const auto t = robot_pose.translation();
    const float angle_rad = std::atan2(robot_pose.linear()(1, 0), robot_pose.linear()(0, 0));
    agv_->robot_poly()->setPos(t.x(), t.y());
    agv_->robot_poly()->setRotation(qRadiansToDegrees(angle_rad));
}

// ─────────────────────────────────────────────────────────────────────────────
// Covariance ellipse
// ─────────────────────────────────────────────────────────────────────────────
void Viewer2D::update_covariance_ellipse(float cx, float cy,
                                         float rx, float ry,
                                         float angle_deg)
{
    if (cov_ellipse_item_ == nullptr)
    {
        cov_ellipse_item_ = agv_->scene.addEllipse(
            -rx, -ry, 2.f * rx, 2.f * ry,
            QPen(QColor(255, 50, 50), 0.03),
            QBrush(QColor(255, 100, 100, 80)));
        cov_ellipse_item_->setZValue(100);
    }
    else
    {
        cov_ellipse_item_->setRect(-rx, -ry, 2.f * rx, 2.f * ry);
    }
    cov_ellipse_item_->setPos(cx, cy);
    cov_ellipse_item_->setRotation(angle_deg);
}

// ─────────────────────────────────────────────────────────────────────────────
// Estimated room rect
// ─────────────────────────────────────────────────────────────────────────────
void Viewer2D::update_estimated_room_rect(float width, float length, bool has_polygon)
{
    if (has_polygon)
    {
        if (estimated_room_item_ != nullptr)
        {
            agv_->scene.removeItem(estimated_room_item_);
            delete estimated_room_item_;
            estimated_room_item_ = nullptr;
        }
        return;
    }

    const QRectF room_rect(-width / 2.f, -length / 2.f, width, length);
    if (estimated_room_item_ == nullptr)
    {
        estimated_room_item_ = agv_->scene.addRect(
            room_rect, QPen(Qt::magenta, 0.05), QBrush(Qt::NoBrush));
        estimated_room_item_->setZValue(2);
    }
    else
    {
        estimated_room_item_->setRect(room_rect);
    }

    fit_view();
}

// ─────────────────────────────────────────────────────────────────────────────
// Room polygon outline
// ─────────────────────────────────────────────────────────────────────────────
void Viewer2D::draw_room_polygon(const std::vector<Eigen::Vector2f>& verts, bool is_capturing)
{
    if (verts.size() < 2)
        return;

    if (polygon_item_ != nullptr)
    {
        agv_->scene.removeItem(polygon_item_);
        delete polygon_item_;
        polygon_item_ = nullptr;
    }

    QPolygonF poly;
    for (const auto& v : verts)
        poly << QPointF(v.x(), v.y());
    if (!is_capturing && verts.size() >= 3)
        poly << QPointF(verts.front().x(), verts.front().y());

    QPen pen(is_capturing ? Qt::yellow : Qt::magenta,
             is_capturing ? 0.08 : 0.15);
    polygon_item_ = agv_->scene.addPolygon(poly, pen, QBrush(Qt::NoBrush));
    polygon_item_->setZValue(8);

    if (!is_capturing)
        fit_view();
}

void Viewer2D::draw_lidar_points(const std::vector<Eigen::Vector3f>& points_high,
                                 const std::vector<Eigen::Vector3f>& points_low,
                                 const Eigen::Affine2f& robot_pose,
                                 int max_points_high)
{
    auto draw_layer = [&](const std::vector<Eigen::Vector3f>& points,
                          std::vector<QGraphicsEllipseItem*>& pool,
                          const QColor& color,
                          int max_points)
    {
        static const qreal radius_px = 1.5;
        const QRectF ellipse_rect(-radius_px, -radius_px, 2 * radius_px, 2 * radius_px);
        QPen pen(color);
        pen.setWidthF(0.0);
        pen.setCosmetic(true);
        QBrush brush(color);

        const int clamped_max_points = std::max(1, max_points);
        const int stride = std::max(1, static_cast<int>(points.size() / clamped_max_points));
        const size_t num_draw = (points.size() + stride - 1) / stride;

        while (pool.size() > num_draw)
        {
            auto* p = pool.back();
            agv_->scene.removeItem(p);
            delete p;
            pool.pop_back();
        }

        size_t idx = 0;
        for (size_t i = 0; i < points.size() && idx < num_draw; i += stride, ++idx)
        {
            const Eigen::Vector2f pr = points[i].head<2>();
            const Eigen::Vector2f pw = robot_pose.linear() * pr + robot_pose.translation();

            if (idx < pool.size())
            {
                pool[idx]->setPos(pw.x(), pw.y());
            }
            else
            {
                auto* item = agv_->scene.addEllipse(ellipse_rect, pen, brush);
                item->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
                item->setPos(pw.x(), pw.y());
                item->setZValue(5);
                pool.push_back(item);
            }
        }
    };

    draw_layer(points_high, lidar_pool_high_, QColor("Green"), max_points_high);
    draw_layer(points_low, lidar_pool_low_, QColor("Cyan"), max_points_high / 2);
}


// ─────────────────────────────────────────────────────────────────────────────
// Composite per-frame update
// ─────────────────────────────────────────────────────────────────────────────
void Viewer2D::update_frame(const FrameData& fd)
{
    const Eigen::Affine2f& lidar_pose = fd.use_loc_pose ? fd.loc_pose : fd.display_pose;
    draw_lidar_points(fd.lidar_points, {}, lidar_pose, fd.max_lidar_points);

    if (fd.have_loc || fd.is_initialized)
        update_robot(fd.display_pose);

    if (fd.have_loc && !fd.has_room_polygon)
        update_estimated_room_rect(fd.room_width, fd.room_length, false);
}

// ─────────────────────────────────────────────────────────────────────────────
// Path
// ─────────────────────────────────────────────────────────────────────────────
void Viewer2D::draw_path(const PathDrawData& data)
{
    clear_path_items();

    if (data.path.size() < 2)
        return;

    // Original polygon vertex dots (green)
    for (const auto& v : data.orig_poly_verts)
    {
        constexpr float r = 0.1f;
        auto* dot = agv_->scene.addEllipse(
            -r, -r, 2.f * r, 2.f * r,
            QPen(QColor(0, 200, 0), 0.02),
            QBrush(QColor(0, 200, 0, 80)));
        dot->setPos(v.x(), v.y());
        dot->setZValue(17);
        path_draw_items_.push_back(dot);
    }

    // Inner (shrunken) polygon outline
    if (navigable_poly_item_ != nullptr)
    {
        agv_->scene.removeItem(navigable_poly_item_);
        delete navigable_poly_item_;
        navigable_poly_item_ = nullptr;
    }
    if (data.inner_poly.size() >= 3)
    {
        QPolygonF qpoly;
        for (const auto& v : data.inner_poly)
            qpoly << QPointF(v.x(), v.y());
        qpoly << QPointF(data.inner_poly.front().x(), data.inner_poly.front().y());

        navigable_poly_item_ = agv_->scene.addPolygon(
            qpoly,
            QPen(QColor(255, 140, 0, 200), 0.03, Qt::DashLine),
            Qt::NoBrush);
        navigable_poly_item_->setZValue(19);
    }

    // Expanded obstacle outlines
    for (auto* item : obstacle_expanded_items_)
    {
        agv_->scene.removeItem(item);
        delete item;
    }
    obstacle_expanded_items_.clear();

    for (const auto& obs : data.expanded_obstacles)
    {
        if (obs.size() < 3)
            continue;

        QPolygonF qpoly;
        for (const auto& v : obs)
            qpoly << QPointF(v.x(), v.y());
        qpoly << QPointF(obs.front().x(), obs.front().y());

        auto* obs_item = agv_->scene.addPolygon(
            qpoly,
            QPen(QColor(255, 140, 0, 200), 0.03, Qt::DashLine),
            Qt::NoBrush);
        obs_item->setZValue(19);
        obstacle_expanded_items_.push_back(obs_item);
    }

    // Navigable polygon vertex dots (yellow)
    for (const auto& v : data.nav_poly)
    {
        constexpr float r = 0.08f;
        auto* dot = agv_->scene.addEllipse(
            -r, -r, 2.f * r, 2.f * r,
            QPen(QColor(255, 255, 0, 200), 0.01),
            QBrush(QColor(255, 255, 0, 120)));
        dot->setPos(v.x(), v.y());
        dot->setZValue(18);
        path_draw_items_.push_back(dot);
    }

    // Path line segments
    const QPen path_pen(QColor(100, 255, 100), 0.04);
    for (std::size_t i = 0; i + 1 < data.path.size(); ++i)
    {
        auto* line = agv_->scene.addLine(
            data.path[i].x(),     data.path[i].y(),
            data.path[i+1].x(),   data.path[i+1].y(),
            path_pen);
        line->setZValue(20);
        path_draw_items_.push_back(line);
    }

    // Intermediate waypoint dots (cyan)
    const QBrush wp_brush(QColor(0, 220, 220));
    for (std::size_t i = 1; i + 1 < data.path.size(); ++i)
    {
        constexpr float r = 0.06f;
        auto* dot = agv_->scene.addEllipse(-r, -r, 2.f * r, 2.f * r, Qt::NoPen, wp_brush);
        dot->setPos(data.path[i].x(), data.path[i].y());
        dot->setZValue(21);
        path_draw_items_.push_back(dot);
    }

    // Goal / target marker
    const auto& goal = data.path.back();
    if (target_marker_ == nullptr)
    {
        constexpr float tr = 0.12f;
        target_marker_ = agv_->scene.addEllipse(
            -tr, -tr, 2.f * tr, 2.f * tr,
            QPen(QColor(255, 50, 50), 0.03),
            QBrush(QColor(255, 50, 50, 120)));
        target_marker_->setZValue(22);
    }
    target_marker_->setPos(goal.x(), goal.y());
    target_marker_->setVisible(true);
}

void Viewer2D::clear_path_items()
{
    for (auto* item : path_draw_items_)
    {
        agv_->scene.removeItem(item);
        delete item;
    }
    path_draw_items_.clear();

    for (auto* item : obstacle_expanded_items_)
    {
        agv_->scene.removeItem(item);
        delete item;
    }
    obstacle_expanded_items_.clear();

    if (target_marker_ != nullptr)
        target_marker_->setVisible(false);
}

void Viewer2D::update_target_marker(float x, float y, bool visible)
{
    if (epistemic_target_item_ == nullptr)
    {
        constexpr float r = 0.10f;
        epistemic_target_item_ = agv_->scene.addEllipse(
            -r, -r, 2.f * r, 2.f * r,
            QPen(QColor(50, 200, 50), 0.03),
            QBrush(QColor(50, 200, 50, 140)));
        epistemic_target_item_->setZValue(25);
    }
    epistemic_target_item_->setPos(x, y);
    epistemic_target_item_->setVisible(visible);
}

// ─────────────────────────────────────────────────────────────────────────────
// Trajectory overlay — draw the selected arc as a polyline with dots
// ─────────────────────────────────────────────────────────────────────────────
void Viewer2D::draw_trajectory(const std::vector<Eigen::Vector3f>& states)
{
    const size_t n = states.size();
    const size_t n_seg = (n > 1) ? n - 1 : 0;

    // Resize line pool
    while (traj_line_items_.size() < n_seg)
    {
        auto* item = agv_->scene.addLine(0, 0, 0, 0,
            QPen(QColor(255, 50, 255), 0.07));
        item->setZValue(35);
        traj_line_items_.push_back(item);
    }
    for (size_t i = 0; i < traj_line_items_.size(); ++i)
        traj_line_items_[i]->setVisible(i < n_seg);

    // Resize dot pool
    while (traj_dot_items_.size() < n)
    {
        constexpr float r = 0.08f;
        auto* item = agv_->scene.addEllipse(-r, -r, 2*r, 2*r,
            QPen(Qt::NoPen), QBrush(QColor(255, 50, 255, 240)));
        item->setZValue(36);
        traj_dot_items_.push_back(item);
    }
    for (size_t i = 0; i < traj_dot_items_.size(); ++i)
        traj_dot_items_[i]->setVisible(i < n);

    // Position items
    for (size_t i = 0; i < n; ++i)
    {
        traj_dot_items_[i]->setPos(states[i][0], states[i][1]);
        if (i > 0)
            traj_line_items_[i - 1]->setLine(
                states[i - 1][0], states[i - 1][1],
                states[i][0], states[i][1]);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void Viewer2D::draw_all_trajectories(
    const std::vector<std::vector<Eigen::Vector3f>>& candidates,
    const std::vector<Eigen::Vector3f>& best)
{
    // Count total candidate segments
    size_t total_segs = 0;
    for (const auto& c : candidates)
        if (c.size() > 1) total_segs += c.size() - 1;

    // Resize candidate line pool
    while (cand_line_items_.size() < total_segs)
    {
        auto* item = agv_->scene.addLine(0, 0, 0, 0,
            QPen(QColor(180, 130, 255, 70), 0.025));
        item->setZValue(33);
        cand_line_items_.push_back(item);
    }
    for (size_t i = 0; i < cand_line_items_.size(); ++i)
        cand_line_items_[i]->setVisible(i < total_segs);

    // Position candidate line segments
    size_t idx = 0;
    for (const auto& c : candidates)
    {
        for (size_t i = 1; i < c.size(); ++i)
        {
            cand_line_items_[idx]->setLine(
                c[i - 1][0], c[i - 1][1],
                c[i][0], c[i][1]);
            ++idx;
        }
    }

    // Draw the best trajectory on top (thick, opaque)
    draw_trajectory(best);
}

// ─────────────────────────────────────────────────────────────────────────────
// Corner detection markers
// ─────────────────────────────────────────────────────────────────────────────
void Viewer2D::draw_corners(const std::vector<rc::CornerDetector::CornerMatch>& matches,
                             const Eigen::Affine2f& robot_pose)
{
    const size_t n = matches.size();

    // Helper: ensure a pool has exactly `count` items, hiding extras
    auto resize_pool = [&](auto& pool, size_t count, auto make_item)
    {
        while (pool.size() < count)
            pool.push_back(make_item());
        for (size_t i = 0; i < pool.size(); ++i)
            pool[i]->setVisible(i < count);
    };

    // Detected corners — solid cyan dots in world frame
    resize_pool(corner_detected_items_, n, [&]() {
        constexpr float r = 0.30f;
        auto* item = agv_->scene.addEllipse(-r, -r, 2*r, 2*r,
            QPen(QColor(0, 220, 255), 0.03), QBrush(QColor(0, 220, 255, 200)));
        item->setZValue(30);
        return item;
    });

    // Predicted corners — hollow red circles in world frame
    resize_pool(corner_predicted_items_, n, [&]() {
        constexpr float r = 0.35f;
        auto* item = agv_->scene.addEllipse(-r, -r, 2*r, 2*r,
            QPen(QColor(255, 140, 0), 0.06), QBrush(Qt::NoBrush));
        item->setZValue(29);
        return item;
    });

    // Lines connecting predicted → detected
    resize_pool(corner_line_items_, n, [&]() {
        QPen pen(QColor(255, 100, 0, 200), 0.03);
        auto* item = agv_->scene.addLine(0, 0, 0, 0, pen);
        item->setZValue(28);
        return item;
    });

    const Eigen::Matrix2f R = robot_pose.linear();
    const Eigen::Vector2f t = robot_pose.translation();

    for (size_t i = 0; i < n; ++i)
    {
        const auto& m = matches[i];

        // Detected: transform from robot frame to world using display pose
        const Eigen::Vector2f det_world = R * m.detected + t;
        // Predicted: use known model world position (exact, no lag)
        const Eigen::Vector2f pred_world = m.model_world;

        corner_detected_items_[i]->setPos(det_world.x(), det_world.y());
        corner_predicted_items_[i]->setPos(pred_world.x(), pred_world.y());
        corner_line_items_[i]->setLine(pred_world.x(), pred_world.y(),
                                        det_world.x(), det_world.y());
    }
}

} // namespace rc
