/*
 *    Copyright (C) 2020 by jvallero & mtorocom
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
	\brief
	@author RoboLab
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsView>
#include <QGraphicsLineItem>
#include <QGraphicsPolygonItem>
#include "grid.h"
#include "navigation.h"
#include "navigation.cpp"
#include <controller.h>
#include <doublebuffer/DoubleBuffer.h>
#include <myscene.h>
#include <Eigen/Dense>
#include <ranges>

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    using Point = std::pair<float, float>;
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);

        void NavigationOptimizer_abort();
        RoboCompNavigationOptimizer::Params NavigationOptimizer_getParams();
        RoboCompNavigationOptimizer::State NavigationOptimizer_getState();
        bool NavigationOptimizer_gotoNewRandomPoint(RoboCompNavigationOptimizer::Params params);

        struct Dimensions
        {
            int TILE_SIZE = 100;
            float HMIN = -2500, VMIN = -2500, WIDTH = 2500, HEIGHT = 2500;
        };

    public slots:
        void compute();
        int startup_check();
        void initialize(int period);

    protected:
        void resizeEvent(QResizeEvent * event)
        {
            graphicsView->fitInView(scene.sceneRect(), Qt::KeepAspectRatio);
        }

    private:
        std::shared_ptr<InnerModel> innerModel;
        std::shared_ptr<RoboCompCommonBehavior::ParameterList> confParams;
        bool startup_check_flag;

        // consts
        const float landa = -0.5 / log(0.1);
        const float MIN_DISTANCE_TO_GOAL = 40;

        //grid
        Dimensions dim;
        using MyGrid = Grid<int, -2500, int, 5000, int, 100>;
        //MyGrid::Dimensions dim;
        //MyGrid grid;

        // robot
        using RobotPose = Navigation<MyGrid, Controller>::RobotPose;
        QPolygonF robot_polygon;
        RobotPose read_base();
        std::tuple<QPolygonF,RoboCompLaser::TLaserData> read_laser();
        const float MAX_SPIKING_ANGLE_rads = 0.2;
        const float MAX_RDP_DEVIATION_mm  =  70;
        void ramer_douglas_peucker(const vector<Point> &pointList, double epsilon, vector<Point> &out);
        void stop_robot();

        // navigation
        Navigation<MyGrid, Controller> navigation;

        //tupla de 3 variables float para las coordenadas x,y,z.
        using Tpose = std::tuple<float, float, float>;

        // Target
        using Target = Navigation<MyGrid,Controller>::Target;
        Target target;
        DoubleBuffer<QPointF, Target> target_buffer;
        MyGrid::Value target_cell;
        bool atTarget = true;

        using Tupla = std::tuple<float, float, float, float, float>;

        //DWA
        std::vector<std::vector<Tupla>> calcularPuntos(float vOrigen, float wOrigen);
        std::optional<Tupla> ordenar(std::vector<Tupla> vector, float x, float y, float rx, float ry, float previous_turn);
        std::vector<Tupla> obstaculos(std::vector<std::vector<Tupla>> vector, float aph, const RoboCompLaser::TLaserData &ldata);
        std::vector<Tupla> dynamicWindowApproach( const RobotPose &robot_pose,
                                                  const RoboCompLaser::TLaserData &ldata,
                                                  const MyGrid::Value &target_cell);
        void fill_grid_with_obstacles();

        // NF
        void navigation_function(QPointF target);

        // draw
        MyScene scene;
        std::vector<QGraphicsItem *> boxes;
        QGraphicsPolygonItem *robot_polygon_draw = nullptr;
        QGraphicsEllipseItem *target_draw = nullptr;
        void init_drawing( Dimensions dim);
        void draw_target(const RobotPose &robot_pose, QPointF t);
        void draw_laser(const QPolygonF &poly);
        void draw_predictions(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata, const std::vector <Tupla> &puntos);
        void initializeWorld();

    void
        draw_things(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata, const std::vector<Tupla> &puntos);



};

#endif
