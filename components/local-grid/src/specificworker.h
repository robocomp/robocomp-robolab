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
//#include <ceres/ceres.h>
#include <fps/fps.h>

using point3f = std::tuple<float, float, float>;

class Viewer : public QGLViewer
{
    public:
        Viewer(QWidget *parent_, std::shared_ptr<std::vector<std::tuple<float, float, float>>> points_,
                                 std::shared_ptr<std::vector<std::tuple<float, float, float>>> colors_,
                                 std::shared_ptr<std::vector<std::tuple<point3f, point3f, point3f, point3f>>> planes_) : QGLViewer(parent_)
        {
            parent = parent_;
            resize(parent->width(), parent->height());
            points = points_;
            colors = colors_;
            planes = planes_;
        };
        ~Viewer(){ saveStateToFile(); }

    protected:
        virtual void draw()
        {
            drawAxis();
            glColor3f(0.7, 0.7, 0.7);
            drawGrid(5.0, 10);
            glBegin(GL_POINTS);
                for (auto &&[point, color] : iter::zip(*points, *colors))
                {
                    auto &[x,y,z] = point;
                    auto &[r,g,b] = color;
                    glColor3f(r, g, b);
                    glVertex3f(x, y, z);
                }
            glEnd();
            glBegin(GL_QUADS);
                glColor3f(1, 0.9, 0.7);
                for (const auto &[p1, p2, p3, p4] : *planes)
                {
                    glVertex3f(std::get<0>(p1), std::get<1>(p1), std::get<2>(p1));
                    glVertex3f(std::get<0>(p2), std::get<1>(p2), std::get<2>(p2));
                    glVertex3f(std::get<0>(p3), std::get<1>(p3), std::get<2>(p3));
                    glVertex3f(std::get<0>(p4), std::get<1>(p4), std::get<2>(p4));
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
        std::shared_ptr<std::vector<std::tuple<float, float, float>>> points, colors;
        std::shared_ptr<std::vector<std::tuple<point3f, point3f, point3f, point3f>>> planes;
        QWidget *parent;
};

class Viewer_Cog : public QGLViewer
{
public:
    Viewer_Cog(QWidget *parent_, std::shared_ptr<std::vector<std::tuple<point3f, point3f, point3f, point3f>>> planes_,
           std::shared_ptr<std::vector<std::tuple<float, float, float>>> colors_) : QGLViewer(parent_)
    {
        parent = parent_;
        resize(parent->width(), parent->height());
        colors = colors_;
        planes = planes_;
    };
    ~Viewer_Cog(){ saveStateToFile(); }

protected:
    virtual void draw()
    {
        drawAxis();
        drawGrid(5.0, 10);
        glBegin(GL_QUADS);
        glColor3f(1, 0.9, 0.7);
        for (const auto &[p1, p2, p3, p4] : *planes)
        {
            glVertex3f(std::get<0>(p1), std::get<1>(p1), std::get<2>(p1));
            glVertex3f(std::get<0>(p2), std::get<1>(p2), std::get<2>(p2));
            glVertex3f(std::get<0>(p3), std::get<1>(p3), std::get<2>(p3));
            glVertex3f(std::get<0>(p4), std::get<1>(p4), std::get<2>(p4));
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
    std::shared_ptr<std::vector<std::tuple<float, float, float>>> points, colors;
    std::shared_ptr<std::vector<std::tuple<point3f, point3f, point3f, point3f>>> planes;
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
    Viewer_Cog *viewer_3d_cog;
    std::shared_ptr<std::vector<point3f>> points, colors, colors_cog;
    std::shared_ptr<std::vector<std::tuple<point3f, point3f, point3f, point3f>>> planes;

    tuple<RoboCompLaser::TLaserData, vector<Eigen::Vector2f>> read_laser();
    std::tuple<cv::Mat, float> read_rgb(const std::string &camera_name);
    std::tuple<cv::Mat, float> read_depth(const string &camera_name);
    cv::Mat read_depth_omni();
    void get_laser_3d_points(const RoboCompLaser::TLaserData &ldata, const std::tuple<float, float, float> &color);
    void get_omni_3d_points(const cv::Mat &depth_frame, const cv::Mat &rgb_frame);
    void get_central_3d_points(const cv::Mat &central_depth, const cv::Mat &central_rgb, float focal);
    Eigen::Transform<float, 3, Eigen::Affine> Rt;

    // array of sets for sectors representation
    struct compare
    { bool operator()(const std::tuple<Eigen::Vector3f, std::tuple<float, float, float>> &a, const std::tuple<Eigen::Vector3f, std::tuple<float, float, float>> &b) const
        { return std::get<Eigen::Vector3f>(a).norm() < std::get<Eigen::Vector3f>(b).norm(); }
    };
    using SetsType = std::vector<std::set<std::tuple<Eigen::Vector3f, std::tuple<float, float, float>>, compare>>;
    SetsType sets;
    SetsType group_by_angular_sectors(bool draw = false);

    // concepts
    QTimer timer_cog;

    // concept tree for rooms
    struct Concept
    {
        bool active=false;
        std::vector<std::string> children;
    };
    std::map<std::string, Concept> room_grammar;

    // floor concept
    vector<Eigen::Vector2f> compute_floor_line(SetsType &sets, bool draw=true);
    cv::RotatedRect estimate_floor_object(const vector<Eigen::Vector2f> &floor_line) const;

    // draw
    void draw_objects(const RoboCompYoloObjects::TObjects &objects, cv::Mat img);
    void draw_people(const RoboCompYoloObjects::TPeople &people, cv::Mat img);

    // objects
    RoboCompYoloObjects::TObjectNames yolo_object_names;
    RoboCompYoloObjects::TJointData yolo_joint_data;

    // FPS
    FPSCounter fps;


};


#endif

//CERES
//struct RectangleCostFunctor
//{
//    template <typename T>
//    bool operator()(const T* const x, T* residual) const
//    {
//        residual[0] = 10.0 - x[0];
//        return true;
//    }
//};