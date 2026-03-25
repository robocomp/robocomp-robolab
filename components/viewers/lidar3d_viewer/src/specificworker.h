/*
 *    Copyright (C) 2023 by YOUR NAME HERE
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

// If you want reduce compute period automaticaly for lack of use
//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include <QGLViewer/qglviewer.h>
#include <cppitertools/zip.hpp>

using point3f = std::tuple<float, float, float>;

class Viewer : public QGLViewer
{
    public:
        Viewer(QWidget *parent_, std::shared_ptr<std::vector<std::tuple<float, float, float>>> points_,
                                 std::shared_ptr<std::vector<std::tuple<float, float, float>>> colors_) : QGLViewer(parent_)
        {
            parent = parent_;
            resize(parent->width(), parent->height());
            points = points_;
            colors = colors_;
        };
        ~Viewer(){ saveStateToFile(); };

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
            resize(parent->width(), parent->height());  // move to a signal
        };
        virtual void init()
        {
            restoreStateFromFile();
            glDisable(GL_LIGHTING);
            glPointSize(3.0);
            setSceneRadius(100.0);
        };
        virtual void animate(){};
        virtual QString helpString() const { return QString();};

    private:
        std::shared_ptr<std::vector<std::tuple<float, float, float>>> points, colors;
        QWidget *parent;

};





class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    public:
        SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
    public slots:
        /**
         * \brief Initializes the worker one time.
         */
        void initialize();

        /**
         * \brief Main compute loop of the worker.
         */
        void compute();

        /**
         * \brief Handles the emergency state loop.
         */
        void emergency();

        /**
         * \brief Restores the component from an emergency state.
         */
        void restore();

        int startup_check();
    private:
        bool startup_check_flag;
        double pc_red;
        double pc_green;
        double pc_blue;
        std::string option = "";

        Viewer *viewer_3d;
        std::shared_ptr<std::vector<point3f>> points, colors;
        std::shared_ptr<std::vector<std::tuple<point3f, point3f, point3f, point3f>>> planes;
        std::string window_name;

    int check_filtered = 1; 
    int check_invalid = 0;
    int slider_start = 180;
    int slider_len = 100;
    int slider_dec = 1;

    static void on_start(int pos,void *data);
    static void on_len(int pos,void *data);
    static void on_decfilter(int pos,void *data);

signals:
        //void customSignal();
};

#endif
