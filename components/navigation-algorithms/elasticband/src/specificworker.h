/*
 *    Copyright (C) 2020 by YOUR NAME HERE
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
	@author authorname
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
#include "grid.cpp"
#include "navigation.h"
#include "navigation.cpp"
#include <controller.h>
#include <doublebuffer/DoubleBuffer.h>
#include <myscene.h>
#include <Eigen/Dense>



class SpecificWorker : public GenericWorker
{
    using Point = std::pair<float, float>;
    Q_OBJECT
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);

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
        std::shared_ptr<InnerModel>innerModel;
        Grid<>::Dimensions dim;
        bool startup_check_flag;
        std::shared_ptr<RoboCompCommonBehavior::ParameterList> confParams;

        // Navigation
        Navigation<Grid<>,Controller> navigation;

        //robot
        RoboCompGenericBase::TBaseState read_base();
        std::tuple<QPolygonF,RoboCompLaser::TLaserData> read_laser();
        const float MAX_SPIKING_ANGLE_rads = 0.2;
        const float MAX_RDP_DEVIATION_mm  =  70;
        void ramer_douglas_peucker(const vector<Point> &pointList, double epsilon, vector<Point> &out);
        void stop_robot();

        // Target
        using Target = Navigation<Grid<>,Controller>::Target;
        Target target;
        DoubleBuffer<QPointF, Target> target_buffer;
        bool atTarget = true;

        // path
        //void draw_path(const std::vector<QPointF> &path);

        // draw
        MyScene scene;
        std::vector<QGraphicsItem *> boxes;
        QGraphicsItem *robot_polygon = nullptr;
        QGraphicsEllipseItem *target_draw = nullptr;
        void init_drawing( Grid<>::Dimensions dim);
        void draw_target(const RoboCompGenericBase::TBaseState &bState, QPointF t);
        void draw_laser(const QPolygonF &poly);
        void initializeWorld();

};

#endif
