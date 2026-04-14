#pragma once

#include <QObject>
#include <QWidget>
#include <QRectF>
#include <QTransform>
#include <QPointF>
#include <QPen>
#include <QBrush>
#include <QColor>
#include <QGraphicsScene>
#include <QGraphicsPolygonItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsRectItem>
#include <QGraphicsTextItem>
#include <Eigen/Dense>
#include <vector>
#include "corner_detector.h"

class AbstractGraphicViewer;

namespace rc {

/**
 * @brief 2D scene viewer for the SLAMO component.
 *
 * Wraps AbstractGraphicViewer and owns all QGraphicsItem* scene objects:
 * robot covariance ellipse, room polygon, furniture polygons, path lines,
 * trajectory debug overlays, and temporary obstacle polygons.
 *
 * All drawing is performed via typed update methods so that SpecificWorker
 * never touches QGraphicsScene directly.
 *
 * Usage:
 *   viewer_2d_ = std::make_unique<rc::Viewer2D>(this->frame, grid_rect, true);
 *   viewer_2d_->add_robot(w, l, 0.0, 0.2, QColor("Blue"));
 *   viewer_2d_->show();
 *   layout->addWidget(viewer_2d_->get_widget());
 *
 *   // per frame:
 *   viewer_2d_->update_robot(robot_pose);
 */
class Viewer2D : public QObject
{
    Q_OBJECT
    public:
        explicit Viewer2D(QWidget* parent, const QRectF& grid_dim, bool show_axis = true);
        ~Viewer2D() override = default;

        // ----- Widget / view management -----
        QWidget* get_widget() const;
        void add_robot(float w, float l, float offset, float rotation, QColor color);
        void show();
        QTransform transform() const;
        void set_transform(const QTransform& t);
        void fit_to_scene(const QRectF& r);
        void fit_view(float margin_ratio = 0.05f);
        void fit_to_robot_and_points(const Eigen::Affine2f& robot_pose,
                         const std::vector<Eigen::Vector3f>& lidar_points,
                         float fit_radius,
                         float margin_ratio = 0.12f);
        void center_on(float x, float y);

        // ----- Robot -----
        void update_robot(const Eigen::Affine2f& robot_pose);

        // ----- Covariance ellipse -----
        void update_covariance_ellipse(float cx, float cy,
                                    float radius_x, float radius_y,
                                    float angle_deg);
        // ----- Estimated room rect (shown when room polygon is absent) -----
        void update_estimated_room_rect(float width, float length, bool has_polygon);

        // ----- Room polygon outline -----
        void draw_room_polygon(const std::vector<Eigen::Vector2f>& verts, bool is_capturing);

        // ----- Lidar -----
        void draw_lidar_points(const std::vector<Eigen::Vector3f>& points_high,
                       const std::vector<Eigen::Vector3f>& points_low,
                       const Eigen::Affine2f& robot_pose,
                       int max_points_high);

        // ----- Composite per-frame update -----
        struct FrameData
        {
            const std::vector<Eigen::Vector3f>& lidar_points;
            Eigen::Affine2f display_pose;
            int max_lidar_points;
            bool have_loc;
            bool is_initialized;
            bool has_room_polygon;
            float room_width;
            float room_length;
            Eigen::Affine2f loc_pose;    // Raw localization pose (no forward projection)
            bool use_loc_pose;           // If true, draw lidar with loc_pose
        };
        void update_frame(const FrameData& fd);
    
        // ----- Path -----
        struct PathDrawData
        {
            std::vector<Eigen::Vector2f> path;
            std::vector<Eigen::Vector2f> orig_poly_verts;
            std::vector<Eigen::Vector2f> inner_poly;
            std::vector<Eigen::Vector2f> nav_poly;
            std::vector<std::vector<Eigen::Vector2f>> expanded_obstacles;
        };
        void draw_path(const PathDrawData& data);
        /// Remove path lines / waypoints / expanded-obstacle overlays; hide target marker.
        void clear_path_items();

        // ----- Epistemic target marker -----
        /// Show a target marker at the given room-frame position, or hide it.
        void update_target_marker(float x, float y, bool visible);

        /// Draw the selected arc trajectory as a polyline on the canvas.
        void draw_trajectory(const std::vector<Eigen::Vector3f>& predicted_states);

        /// Draw all candidate trajectories as faded polylines, then the best one on top.
        void draw_all_trajectories(const std::vector<std::vector<Eigen::Vector3f>>& candidates,
                                   const std::vector<Eigen::Vector3f>& best);

        /// Draw detected corners: green circles for accepted, yellow for predicted/in-FOV
        void draw_corners(const std::vector<rc::CornerDetector::CornerMatch>& matches,
                          const Eigen::Affine2f& robot_pose);

        /// Draw the epistemic score grid as semi-transparent coloured cells.
        /// cell_size is in world-frame meters.
        void draw_score_grid(const std::vector<std::pair<Eigen::Vector2f, float>>& cells,
                             float cell_size);

    Q_SIGNALS:
        void robot_moved(QPointF);
        void robot_rotate(QPointF);
        void robot_dragging(QPointF);
        void robot_drag_end(QPointF);
        void new_mouse_coordinates(QPointF);
        void right_click(QPointF);

    private:
        AbstractGraphicViewer* agv_ = nullptr;

        // Covariance ellipse
        QGraphicsEllipseItem* cov_ellipse_item_ = nullptr;

        // Estimated room rect (shown when no polygon is loaded)
        QGraphicsRectItem* estimated_room_item_ = nullptr;

        // Room polygon capture
        std::vector<QGraphicsEllipseItem*> capture_vertex_items_;

        // Room polygon outline
        QGraphicsPolygonItem* polygon_item_         = nullptr;
        QGraphicsPolygonItem* polygon_item_backup_  = nullptr;

        // Furniture
        std::vector<QGraphicsPolygonItem*> furniture_draw_items_;

        // Temporary obstacles
        std::vector<QGraphicsPolygonItem*> temp_obstacle_draw_items_;

        // Lidar point pools (reused each frame)
        std::vector<QGraphicsEllipseItem*> lidar_pool_high_;
        std::vector<QGraphicsEllipseItem*> lidar_pool_low_;

        // Path
        std::vector<QGraphicsItem*>         path_draw_items_;
        QGraphicsEllipseItem*               target_marker_        = nullptr;
        QGraphicsPolygonItem*               navigable_poly_item_  = nullptr;
        std::vector<QGraphicsPolygonItem*>  obstacle_expanded_items_;

        // Epistemic target
        QGraphicsEllipseItem* epistemic_target_item_ = nullptr;

        // Corner detection markers
        std::vector<QGraphicsEllipseItem*> corner_detected_items_;
        std::vector<QGraphicsEllipseItem*> corner_predicted_items_;
        std::vector<QGraphicsLineItem*>    corner_line_items_;

        // Trajectory overlay
        std::vector<QGraphicsLineItem*>   traj_line_items_;
        std::vector<QGraphicsEllipseItem*> traj_dot_items_;

        // Candidate trajectory overlay (faded)
        std::vector<QGraphicsLineItem*>   cand_line_items_;

        // Score grid overlay
        std::vector<QGraphicsRectItem*> score_grid_items_;

    };

} // namespace rc
