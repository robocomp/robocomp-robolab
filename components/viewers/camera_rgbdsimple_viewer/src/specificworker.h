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
#include <librealsense2/rs.hpp>
#include <innermodel/innermodel.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <fps/fps.h>

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
	std::shared_ptr < InnerModel > innerModel;
	bool startup_check_flag;

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    std::unique_ptr<rs2::align> align;
    rs2::pipeline_profile profile;
    rs2::frame rgb_frame, depth_frame;
    std::string camera_name;
    bool display_rgb = false;
    bool display_depth = false;
    bool display_compressed = false;

    FPSCounter fps;

};

#endif
