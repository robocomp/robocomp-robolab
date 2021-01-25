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
{
};

struct TContDefault
{
    TContDefault(){};
};

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

        // consts
        float KE = 10;
        float KI = 300;
        float MAX_LASER_DIST = 4000;
        float MAX_DIST_TO_FORCE = 1200; //mm

        TMap grid;
        TController controller;

        void initialize(const Grid<>::Dimensions &dim, std::shared_ptr<InnerModel> innerModel_,
                        std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams_,
                        RoboCompOmniRobot::OmniRobotPrxPtr omnirobot_proxy_,
                        QGraphicsScene *scene_);
        void update(const RoboCompLaser::TLaserData &laser_data, Target &target, bool needsReplaning);

    private:
        std::shared_ptr<Collisions> collisions;
        std::shared_ptr<InnerModel> innerModel;
        std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams;
        RoboCompOmniRobot::OmniRobotPrxPtr omnirobot_proxy;

        // ElasticBand
        std::vector<QPointF> pathPoints;

        // Draw
        QGraphicsScene *scene;

        // Robot
        float ROBOT_LENGTH = 500;
        float ROBOT_WIDTH = 500; // values read from config in initialize()
        const float ROAD_STEP_SEPARATION = ROBOT_LENGTH * 0.7;
        QVec robotBottomLeft, robotBottomRight, robotTopRight, robotTopLeft;
        QPolygonF robot_polygon_r;
        const float OFF = 100; // Offset for extended robot polygon used in bumper control

         // Integrating time
        QTime reloj = QTime::currentTime();

        // Social
        bool gridChanged = false;
        vector<QPolygonF> intimateSpaces, personalSpaces, socialSpaces, totalAffordances;
        vector<QPolygonF> affordancesBlocked;
        std::map<float, vector<QPolygonF>> mapCostObjects;

        ////////// ELASTIC BAND RELATED METHODS //////////////////////////////////
        void compute_forces( std::vector<QPointF> &path,
                             const vector<QPointF> &laser_cart,
                             const QPolygonF &laser_poly,
                             const QPolygonF &current_robot_polygon,
                             const QPointF &current_robot_nose);
        void clean_points(std::vector<QPointF> &path, const QPolygonF &laser_poly, const QPolygonF &current_robot_polygon);
        void add_points(std::vector<QPointF> &path, const QPolygonF &laser_poly, const QPolygonF &current_robot_polygon);
        void stopRobot();
        /////////// AUX //////////////////////////////////////////////////////////////////
        std::tuple<QPolygonF, std::vector<QPointF>> read_laser_data(const RoboCompLaser::TLaserData &laser_data);
        bool is_point_visitable(QPointF point);
        bool is_visible(QPointF p, const QPolygonF &laser_poly);
        QPolygonF get_robot_polygon();

        ////////// GRID RELATED METHODS //////////
        void updateFreeSpaceMap(bool drawGrid = true);
        void draw_path(const std::vector<QPointF> &path, QGraphicsScene *scene);
};
#endif //PROJECT_NAVIGATION_