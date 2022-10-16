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
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <PAL/PAL.h>
#include <PAL/PAL_CameraProperties.h>
#include <X11/Xlib.h>
#include <chrono>
#include <doublebuffer/DoubleBuffer.h>
#include <fps/fps.h>

namespace PAL
{
	namespace Internal
	{
		void EnableDepth(bool flag);
		void MinimiseCompute(bool flag);
	}
}

class SpecificWorker : public GenericWorker
{

	Q_OBJECT
	public:
		SpecificWorker(TuplePrx tprx, bool startup_check);
		~SpecificWorker();
		bool setParams(RoboCompCommonBehavior::ParameterList params);

		RoboCompCameraRGBDSimple::TRGBD CameraRGBDSimple_getAll(std::string camera);
		RoboCompCameraRGBDSimple::TDepth CameraRGBDSimple_getDepth(std::string camera);
		RoboCompCameraRGBDSimple::TImage CameraRGBDSimple_getImage(std::string camera);
		RoboCompCameraRGBDSimple::TPoints CameraRGBDSimple_getPoints(std::string camera);


public slots:
		void compute();
		int startup_check();
		void initialize(int period);

	private:
		bool startup_check_flag;

		Display* disp;
		Screen* scrn;
		int sc_height;
		int sc_width ;
		DoubleBuffer<PAL::Image, RoboCompCameraRGBDSimple::TImage> image_buffer;
		DoubleBuffer<PAL::Image, RoboCompCameraRGBDSimple::TDepth> depth_buffer;
		DoubleBuffer<std::vector<PAL::Point>, RoboCompCameraRGBDSimple::TPoints> points_buffer;
		FPSCounter fps;

};

#endif
