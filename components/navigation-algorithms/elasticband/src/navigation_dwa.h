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
#include "innerviewer.h"
#include <cppitertools/chain.hpp>
#include <cppitertools/range.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/zip.hpp>
#include <cppitertools/filter.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/slice.hpp>
#include <algorithm>
#include <localPerson.h>
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

        // Draw
        QGraphicsScene *scene;
        void draw_path(const std::vector<QPointF> &path, QGraphicsScene *scene);

        // Robot
        float ROBOT_LENGTH = 500;
        float ROBOT_WIDTH = 500; // values read from config in initialize()
        QVec robotBottomLeft, robotBottomRight, robotTopRight, robotTopLeft;
        QPolygonF robot_polygon_r;
        const float OFF = 100; // Offset for extended robot polygon used in bumper control


        ////////// DWA //////////////////////////////////


        /////////// AUX //////////////////////////////////////////////////////////////////
        std::tuple<QPolygonF, std::vector<QPointF>> read_laser_data(const RoboCompLaser::TLaserData &laser_data);
        QPolygonF get_robot_polygon();

};
#endif //PROJECT_NAVIGATION_