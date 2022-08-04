/*
 *    Copyright (C) 2022 by YOUR NAME HERE
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
#include "/home/robocomp/robocomp/classes/local_grid/local_grid.h"
#include "/home/robocomp/robocomp/classes/abstract_graphic_viewer/abstract_graphic_viewer.h"
#include "qgraphicscellitem.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <QGLViewer/qglviewer.h>
#include <cppitertools/zip.hpp>

class Viewer : public QGLViewer
{
    public:
        Viewer(QWidget *parent_, std::shared_ptr<std::vector<std::tuple<double, double, double>>> points_,
                                 std::shared_ptr<std::vector<std::tuple<double, double, double>>> colors_) : QGLViewer(parent_)
        {
            parent = parent_;
            resize(parent->width(), parent->height());
            points = points_;
            colors = colors_;
        };
        ~Viewer(){ saveStateToFile(); }

    protected:
        virtual void draw()
        {
            drawAxis();
            drawGrid(5.0, 10);
            glBegin(GL_POINTS);
            for (auto &&[point, color] : iter::zip(*points, *colors))
            {
                auto &[x,y,z] = point;
                auto &[r,g,b] = color;
                glColor3f(r, g, b);
                glVertex3fv(qglviewer::Vec(x, y, z));
            }
            glEnd();
            resize(parent->width(), parent->height());  // move to a signal
        };
        virtual void init()
        {
            restoreStateFromFile();
            glDisable(GL_LIGHTING);
            glPointSize(3.0);
            setSceneRadius(5.0);
            //setGridIsDrawn();
        };
        virtual void animate(){};
        virtual QString helpString() const { return QString();};

    private:
        std::shared_ptr<std::vector<std::tuple<double, double, double>>> points, colors;
        QWidget *parent;
};

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);


    public slots:
        void compute();
        int startup_check();
        void initialize(int period);

    private:

    struct Constants
    {
        float tile_size = 100;
        const float max_laser_range = 4000;
        const float max_camera_depth_range = 4500;
        const float min_camera_depth_range = 800;
        const float omni_camera_height_meters = 0.6; //mm
        float robot_length = 500;
     };
    Constants consts;

    bool startup_check_flag;
    AbstractGraphicViewer *viewer;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsRectItem *laser_in_robot_polygon;
    QRectF viewer_dimensions;

    Local_Grid local_grid;

    Viewer *viewer_3d;
    std::shared_ptr<std::vector<std::tuple<double, double, double>>> points, colors;

    RoboCompLaser::TLaserData read_laser();
    cv::Mat read_rgb(const std::string &camera_name);
    cv::Mat read_depth_omni();
    void draw_laser_on_3dviewer(const RoboCompLaser::TLaserData &ldata);
    void draw_omni_depth_frame_on_3dviewer(const cv::Mat &depth_frame, const cv::Mat &rgb_frame);
};

#endif
