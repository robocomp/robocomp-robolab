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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    display_rgb = (params["display_rgb"].value == "true") or (params["display_rgb"].value == "True");
    display_depth = (params["display_depth"].value == "true") or (params["display_depth"].value == "True");
    return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

    try
    {
        cfg.enable_device(serial);
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        //pipe(ctx);
        pipe.start(cfg);
        cam_intr = pipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
        depth_intr = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();

        filters.emplace_back("Decimate", dec_filter);
        //filters.emplace_back(disparity_filter_name, depth_to_disparity);
        filters.emplace_back("Spatial", spat_filter);
        filters.emplace_back("Temporal", temp_filter);
        filters.emplace_back("HFilling", holef_filter);
    }
    catch(std::exception &e)
    { std::cout<<e.what()<<std::endl; }

	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{

    rs2::frameset data = pipe.wait_for_frames();
    depth = data.get_depth_frame(); // Find and colorize the depth data

    for (auto &&filter : filters)
        if (filter.is_enabled)
            depth = filter.filter.process(depth);

    rgb = data.get_color_frame(); // Find the color data

    if (display_rgb)
    {
        cv::Mat frame(cv::Size(cam_intr.width, cam_intr.height), CV_8UC3, (void*)rgb.get_data(), cv::Mat::AUTO_STEP);
        cv::imshow("RGB image", frame);
        cv::waitKey(1);
    }
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

RoboCompCameraRGBDSimple::TRGBD SpecificWorker::CameraRGBDSimple_getAll(std::string camera)
{
//implementCODE

}

RoboCompCameraRGBDSimple::TDepth SpecificWorker::CameraRGBDSimple_getDepth(std::string camera)
{
//implementCODE

}

RoboCompCameraRGBDSimple::TImage SpecificWorker::CameraRGBDSimple_getImage(std::string camera)
{
//implementCODE

}



/**************************************/
// From the RoboCompCameraRGBDSimplePub you can publish calling this methods:
// this->camerargbdsimplepub_pubproxy->pushRGBD(...)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

