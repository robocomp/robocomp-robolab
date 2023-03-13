/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cppitertools/enumerate.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cppitertools/range.hpp>
#include <fps/fps.h>
#include <cmath>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

    RoboCompLaser::TLaserData Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState);
    RoboCompLaser::LaserConfData Laser_getLaserConfData();
    RoboCompLaser::TLaserData Laser_getLaserData();

public slots:
	void compute();
	int startup_check();
	void initialize(int period);

private:

    const int max_dist = 5000; //millimeters
    const int lado = 500; // window siae in pixels
    const float scale = (float)lado/max_dist;  // pixels per millimeter
    const int semilado = lado/2;
    const float radius = 500; // circle separation

    bool startup_check_flag;
    RoboCompLaser::TLaserData new_laser;
    void draw_laser(const RoboCompLaser::TLaserData &ldata);
    std::mutex my_mutex;
    FPSCounter fps;
    cv::Mat robot_image;
};

#endif
