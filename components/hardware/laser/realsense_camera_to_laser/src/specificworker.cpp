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
#include <cppitertools/sliding_window.hpp>

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
    // get periodd, camera ID, fps, size
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

    try
    {
        cfg.enable_device(serial);
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        profile = pipe.start(cfg);

        // Each depth camera might have different units for depth pixels, so we get it here
        // Using the pipeline's profile, we can retrieve the device that the pipeline uses
        depth_scale = get_depth_scale(profile.get_device());

        depth_intr = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();

        rs2_error** error;
        rs2_set_option(dec_filter, RS2_OPTION_FILTER_MAGNITUDE, 4, error);
        filters.emplace_back("Decimate", dec_filter);
        //        rs2::disparity_transform depth_to_disparity(true);
        //        rs2::disparity_transform disparity_to_depth(false);
        //        filters.emplace_back("Disparity", depth_to_disparity);
        filters.emplace_back("Spatial", spat_filter);
        filters.emplace_back("Temporal", temp_filter);
        filters.emplace_back("HFilling", holef_filter);
    }
    catch(std::exception &e)
    { std::cout<<e.what()<<std::endl; }

    this->Period = period;
	if(this->startup_check_flag)
		this->startup_check();
	else
		timer.start(Period);
}

void SpecificWorker::compute()
{
    rs2::frameset frameset = pipe.wait_for_frames();
    rs2::frame depth_frame = frameset.get_depth_frame();
    //If one of them is unavailable, continue iteration
    if (not depth_frame)
        return;

    bool revert_disparity = false;
    for (auto &&filter : filters)
            depth_frame = filter.filter.process(depth_frame);

    auto points = pointcloud.calculate(depth_frame);
    auto ldata = compute_laser(points);

    if (true)
    {
        rs2::frame depth_color = depth_frame.apply_filter(color_map);

        // Query frame size (width and height)
        const int w = depth_frame.as<rs2::video_frame>().get_width();
        const int h = depth_frame.as<rs2::video_frame>().get_height();

        cv::Mat frame_depth(cv::Size(w, h), CV_8UC3, (void*)depth_color.get_data(), cv::Mat::AUTO_STEP);
        cv::imshow("DEPTH image", frame_depth);
        cv::waitKey(1);
    }

    cv::Mat laser_img(cv::Size(400, 400), CV_8UC3);
    laser_img = cv::Scalar(255,255,255);
    float scale = 100;
    float x = ldata.front().dist * sin(ldata.front().angle) * scale + 200;
    float y = 400 - ldata.front().dist * cos(ldata.front().angle) * scale;
    cv::line(laser_img, cv::Point{200,400}, cv::Point(x,y), cv::Scalar(0,200,0));
    for(auto &&l : iter::sliding_window(ldata, 2))
    {
        float x1 = l[0].dist * sin(l[0].angle) * scale + 200;
        float y1 = 400 - l[0].dist * cos(l[0].angle) * scale;
        float x2 = l[1].dist * sin(l[1].angle) * scale + 200;
        float y2 = 400 - l[1].dist * cos(l[1].angle) * scale;
        cv::line(laser_img, cv::Point{x1,y1}, cv::Point(x2,y2), cv::Scalar(0,200,0));
    }
    x = ldata.back().dist * sin(ldata.back().angle) * scale + 200;
    y = 400 - ldata.back().dist * cos(ldata.back().angle) * scale;
    cv::line(laser_img, cv::Point(x,y), cv::Point(200,400), cv::Scalar(0,200,0));

    cv::imshow("Laser", laser_img);
    cv::waitKey(2);

    fps.print("FPS: ");
}

RoboCompLaser::TLaserData SpecificWorker::compute_laser(const rs2::points &points)
{
    const int MAX_LASER_BINS = 100;
    const float TOTAL_HOR_ANGLE = 1.04;  // rads para 60º
    using Point = std::tuple<float, float, float>;
    auto cmp = [](Point a, Point b) {
        auto &[ax, ay, az] = a;
        auto &[bx, by, bz] = b;
        return (ax * ax + ay * ay + az * az) < (bx * bx + by * by + bz * bz);
    };
    std::vector<std::set<Point, decltype(cmp) >> hor_bins(MAX_LASER_BINS);

    const rs2::vertex *vertices = points.get_vertices();
    for (size_t i = 0; i < points.size(); i++)
    {
        if(vertices[i].z >= 0.1)
        {
            float xv = vertices[i].x;
            float yv = vertices[i].y;
            float zv = vertices[i].z;
            if (fabs(yv) < 0.2)
            {
                float hor_angle = atan2(xv, zv);
                // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
                int angle_index = (int) ((MAX_LASER_BINS / TOTAL_HOR_ANGLE) * hor_angle + (MAX_LASER_BINS / 2));
                if (angle_index >= 100 or angle_index < 0) continue;
                hor_bins[angle_index].emplace(std::make_tuple(xv, yv, zv));
            }
        }
    }

    RoboCompLaser::TLaserData ldata(MAX_LASER_BINS);
    uint i = 0;
    for (auto &bin : hor_bins)
    {
        ldata[i].angle = (i - MAX_LASER_BINS / 2.f) / (MAX_LASER_BINS / TOTAL_HOR_ANGLE);
        if (bin.size() > 0)
        {
            const auto &[X, Y, Z] = *bin.cbegin();
            ldata[i].dist = sqrt(X * X + Y * Y + Z * Z);
        }
        else
            ldata[i].dist = 0.f;
        i++;
    }
    return ldata;
}

float SpecificWorker::get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
            return dpt.get_depth_scale();
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

///////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState)
{


}

RoboCompLaser::LaserConfData SpecificWorker::Laser_getLaserConfData()
{
//implementCODE

}

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserData()
{
//implementCODE

}




/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

