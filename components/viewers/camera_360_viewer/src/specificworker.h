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

#include <genericworker.h>
#include <fps/fps.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);
        struct Fovea
        {
            int cx; int cy; int width=300; int height=300;
            int max_width; int max_height;
            float r2full_x, r2full_y;
        };

    public slots:
        void compute();
        int startup_check();
        void initialize(int period);


    private:
        bool startup_check_flag;
        FPSCounter fps;
        static int slider_width, slider_height, slider_x, slider_y;
        RoboCompCamera360RGB::TImage initial_img;
        int MAIN_IMAGE_WIDTH = 640;
        int MAIN_IMAGE_HEIGHT = 640;
        static Fovea fovea;
        std::chrono::time_point<std::chrono::high_resolution_clock> begin;

    static void on_cx(int pos, void *data);
        static void on_cy(int pos, void *data);
        static void on_width(int pos, void *data);
        static void on_height(int pos, void *data);
        void adjustPeriod(int new_period);
};

#endif
