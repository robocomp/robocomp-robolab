//
// Created by robolab on 17/01/20.
//

#ifndef PROJECT_NAVIGATION_H
#define PROJECT_NAVIGATION_H



#include <innermodel/innermodel.h>
#include <math.h>
#include <genericworker.h>
#include <Laser.h>
#include "collisions.h"
#include <QPolygonF>
#include <QPointF>
#include <cppitertools/chain.hpp>
#include <cppitertools/range.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/zip.hpp>
#include <cppitertools/filter.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/slice.hpp>
#include <algorithm>
#include <typeinfo>
#include <Eigen/Dense>
#include <grid.h>
#include <controller.h>

// Map
struct TMapDefault
{};

struct TContDefault
{
    TContDefault(){};
};

using Tupla = std::tuple<float, float, float, float, float>;

template<typename TMap = TMapDefault, typename TController = TContDefault>
class Navigation
{
    public:
        // Target
        struct Target
        {
            QPointF pos;
            float ang;
        };
        struct RobotPose
        {
            float x; float y;
            float ang;
            float x_vel; float y_vel;
            float ang_vel;
            QGraphicsPolygonItem *robot_polygon_draw;
        };

        // consts
        float MAX_LASER_DIST = 4000;

        TMap grid;
        TController controller;

        void initialize( std::shared_ptr<InnerModel> innerModel_,
                         std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams_,
                         RoboCompOmniRobot::OmniRobotPrxPtr omnirobot_proxy_,
                         QGraphicsScene *scene_, QPolygonF robot_polygon_);
        void update( const RobotPose &robot_pose, const QPolygonF &laser_polygon, Target &target);
        bool plan_new_path(QPointF target_point);

private:
        std::shared_ptr<Collisions> collisions;
        std::shared_ptr<InnerModel> innerModel;
        std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams;
        RoboCompOmniRobot::OmniRobotPrxPtr omnirobot_proxy;

        // Draw
        QGraphicsScene *scene;
        std::vector<QGraphicsEllipseItem *> arcs_vector;
        void draw_predictions( const RobotPose &robot_pose, const std::vector <Tupla> &puntos);

        // Robot
        QVec robotBottomLeft, robotBottomRight, robotTopRight, robotTopLeft;
        QPolygonF robot_polygon_r;
        const float OFF = 100; // Offset for extended robot polygon used in bumper control

        ////////// ELASTIC BAND RELATED METHODS //////////////////////////////////
        std::vector<std::vector<Tupla>> compute_potential_set(float vOrigen, float wOrigen);
        std::vector<Tupla> remove_obstacle_hits( const std::vector<std::vector<Tupla>> &vector_arcs,
                                                 const QPolygonF &laser_polygon,
                                                 const QPolygonF &robot_polygon_r);
        void stopRobot();
        std::optional<Tupla> cost_function(std::vector<Tupla> vector, float x, float y, float rx, float ry, float previous_turn);

        /////////// AUX //////////////////////////////////////////////////////////////////
        std::tuple<QPolygonF, std::vector<QPointF>> read_laser_data(const RoboCompLaser::TLaserData &laser_data);
        QPolygonF get_robot_polygon();



};
#endif //PROJECT_NAVIGATION_