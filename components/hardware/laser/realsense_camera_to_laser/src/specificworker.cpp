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
#include <cppitertools/range.hpp>

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
    serial_left = params["serial_left"].value;
    serial_right = params["serial_right"].value;
    serial_center = params["serial_center"].value;
    display_depth = (params["display_depth"].value == "true") or (params["display_depth"].value == "True");
    display_rgb = (params["display_rgb"].value == "true") or (params["display_rgb"].value == "True");
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initializing worker" << std::endl;

    try
    { cfg_center.enable_device(serial_center);}
    catch(const std::exception &e)
    { std::cout << e.what()<< " Serial number:" << serial_center << std::endl; std::terminate();}
    try
    { cfg_left.enable_device(serial_left);}
    catch(const std::exception &e)
    { std::cout << e.what()<< " Serial number:" << serial_left << std::endl; std::terminate();}
    try
    { cfg_right.enable_device(serial_right);}
    catch(const std::exception &e)
    { std::cout << e.what()<< " Serial number:" << serial_right << std::endl; std::terminate();}

    try
    {
        const int w = 640;
        const int h = 480;
        cfg_center.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, 15);
        cfg_left.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, 15);
        cfg_right.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, 15);

        rs2::pipeline center_pipe;
        rs2::pipeline_profile profile_center = center_pipe.start(cfg_center);
        center_depth_intr = center_pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
        float center_fov[2]; // X, Y fov
        rs2_fov(&center_depth_intr, center_fov);
        std::cout << "Camera " << serial_center << " started" << std::endl;
        std::cout << "  width: " << center_depth_intr.width << std::endl;
        std::cout << "  height: " << center_depth_intr.height << std::endl;
        std::cout << "  image center x: " << center_depth_intr.ppx << std::endl;
        std::cout << "  image center y: " << center_depth_intr.ppy << std::endl;
        std::cout << "  focal x: " << center_depth_intr.fx << std::endl;
        std::cout << "  focal y: " << center_depth_intr.fy << std::endl;
        std::cout << "  horizontal angle: " << center_fov[0] << std::endl;
        std::cout << "  vertical angle: " << center_fov[1] << std::endl;
        for (auto p : profile_center.get_streams())
            std::cout << "  stream ID: " << p.unique_id() << " - Stream name: " << p.stream_name() << std::endl;

        const float ALTURA_AL_SUELO = 1.0;
        Eigen::Translation<float, 3> center_tr(0.f, ALTURA_AL_SUELO, 0.100);
        Eigen::Matrix3f center_m;
        center_m = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX())
                 * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY())
                 * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ());
        Eigen::Transform<float, 3, Eigen::Affine> center_depth_extrinsics;;
        center_depth_extrinsics = center_tr;
        center_depth_extrinsics.rotate(center_m);
        std::cout << "center transform " << center_depth_extrinsics.matrix() << std::endl;
        cam_map[serial_center] = std::make_tuple(center_pipe, center_depth_intr, center_depth_extrinsics, rs2::frame(), rs2::points());
        qInfo() << __FUNCTION__ << " center-camera started";

        // right
        rs2::pipeline right_pipe;
        right_pipe.start(cfg_right);
        right_depth_intr = right_pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
        Eigen::Translation<float, 3> right_tr(0.0963, ALTURA_AL_SUELO, 0.0578);
        Eigen::Matrix3f right_m;
        right_m = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(M_PI/3, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ());
        Eigen::Transform<float, 3, Eigen::Affine> right_depth_extrinsics;;
        right_depth_extrinsics = right_tr;
        right_depth_extrinsics.rotate(right_m);
        std::cout << "right transform " << right_depth_extrinsics.matrix() << std::endl;
        cam_map[serial_right] = std::make_tuple(right_pipe, right_depth_intr, right_depth_extrinsics,  rs2::frame(), rs2::points());
        qInfo() << __FUNCTION__ << " right-camera started";

        // left
        rs2::pipeline left_pipe;
        left_pipe.start(cfg_left);
        left_depth_intr = left_pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
        Eigen::Translation<float, 3> left_tr(-0.0963, ALTURA_AL_SUELO, 0.0578);
        Eigen::Matrix3f left_m;
        left_m = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX())
                * Eigen::AngleAxisf(-M_PI/3, Eigen::Vector3f::UnitY())
                * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ());
        Eigen::Transform<float, 3, Eigen::Affine> left_depth_extrinsics;;
        left_depth_extrinsics = left_tr;
        left_depth_extrinsics.rotate(left_m);
        std::cout << "left transform " << left_depth_extrinsics.matrix() << std::endl;
        cam_map[serial_left] = std::make_tuple(left_pipe, left_depth_intr, left_depth_extrinsics, rs2::frame(), rs2::points());
        qInfo() << __FUNCTION__ << " left-camera started";

        // Filters
        rs2_set_option(dec_filter, RS2_OPTION_FILTER_MAGNITUDE, 4, error);
        filters.emplace_back("Decimate", dec_filter);
        filters.emplace_back("Spatial", spat_filter);
        filters.emplace_back("Temporal", temp_filter);
        filters.emplace_back("HFilling", holef_filter);
    }
    catch(const std::exception &e)
    { std::cout << e.what() << " - Exception at pipe start" << std::endl; std::terminate();}


    this->Period = 50;
	if(this->startup_check_flag)
		this->startup_check();
	else
		timer.start(Period);
}

void SpecificWorker::compute()
{
    const auto &cam_map_extended = read_and_filter(cam_map);   // USE OPTIONAL
    auto ldata_local = compute_laser(cam_map_extended);

    if ( display_depth )
    {
        std::map<std::string,cv::Mat> image_stack;
        int w, h;
        for (auto &[key, value] : cam_map)
        {
            auto &[pipe, intr, extr, depth_frame, points] = value;
            if(depth_frame)
            {
                rs2::frame depth_color = depth_frame.apply_filter(color_map);
                w = depth_frame.as<rs2::video_frame>().get_width();
                h = depth_frame.as<rs2::video_frame>().get_height();
                cv::Mat frame_depth(cv::Size(w,h), CV_8UC3, (void*)depth_color.get_data(), cv::Mat::AUTO_STEP);
                //image_stack.emplace_back(cv::Mat(cv::Size(w,h), CV_8UC3, (void*)depth_color.get_data(), cv::Mat::AUTO_STEP));
                image_stack[key] = frame_depth.clone();
            }
        }
        cv::Mat frame_final(cv::Size(w * image_stack.size(), h), CV_8UC3);
        image_stack[serial_left].copyTo(frame_final(cv::Rect(0,0,w,h)));
        image_stack[serial_center].copyTo(frame_final(cv::Rect(w,0,w,h)));
        image_stack[serial_right].copyTo(frame_final(cv::Rect(2*w,0,w,h)));
//        for(auto &&[i, img] : iter::enumerate(image_stack))
//            img.copyTo(frame_final(cv::Rect(i*w,0,w,h)));
        cv::imshow("Depth mosaic", frame_final);
        cv::waitKey(1);
    }
    draw_laser(ldata);
    fps.print("FPS: ");

    std::scoped_lock lock(my_mutex);
    ldata.swap(ldata_local);
}

SpecificWorker::Camera_Map& SpecificWorker::read_and_filter(Camera_Map &cam_map)
{
    for (auto &[key, value] : cam_map)
    {
        //if(key != serial_center and key != serial_right) continue;

        auto &[my_pipe, intr, extr, depth_frame, points] = value;
        rs2::frameset data = my_pipe.wait_for_frames();
        depth_frame = data.get_depth_frame(); // Find and colorize the depth dat

        for (auto &&filter : filters)
            depth_frame = filter.filter.process(depth_frame);

        // rgb_list[i] = data.get_color_frame(); // Find the color data
        rs2::pointcloud pointcloud;
        points = pointcloud.calculate(depth_frame);
        // pointclouds[i].map_to(rgb_list[i]);
    }
    return cam_map;
}

RoboCompLaser::TLaserData SpecificWorker::compute_laser(const Camera_Map &cam_map_extended)
{
    const int MAX_LASER_BINS = 200;
    const float TOTAL_HOR_ANGLE = M_PI;  // rads para 180º
    using Point = std::tuple<float, float, float>;
    auto cmp = [](Point a, Point b)
    {
        auto &[ax, ay, az] = a;
        auto &[bx, by, bz] = b;
        return (ax * ax + ay * ay + az * az) < (bx * bx + by * by + bz * bz);
    };
    std::vector<std::set<Point, decltype(cmp) >> hor_bins(MAX_LASER_BINS);

    for( const auto &[key, value] : cam_map_extended)
    {
        const auto &[pipe, intrin, extrin, depth_frame, points] = value;
        if(points.size() == 0) continue;
        const rs2::vertex *vertices = points.get_vertices();
        float FLOOR_DISTANCE_MINUS_OFFSET =  extrin.translation().y() * 0.9;  // Y axis points downwards
        for (size_t i = 0; i < points.size(); i++)
        {
            if(vertices[i].z >= 0.1)
            {
                auto to_point = extrin * Eigen::Vector3f{vertices[i].x, vertices[i].y, vertices[i].z};
                const float &xv = to_point[0]; const float &yv = to_point[1]; const float &zv = to_point[2];

//                std::cout << from_point[0] << " " << from_point[1] << " " << from_point[2] << std::endl;
//                std::cout << to_point[0] << " " << to_point[1] << " " << to_point[2] << std::endl;
//                std::cout << std::endl;

                if (yv < FLOOR_DISTANCE_MINUS_OFFSET)
                {
                    float hor_angle = atan2(xv, zv);
                    // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
                    int angle_index = (int) ((MAX_LASER_BINS / TOTAL_HOR_ANGLE) * hor_angle + (MAX_LASER_BINS / 2));
                    if (angle_index >= MAX_LASER_BINS or angle_index < 0) continue;
                    hor_bins[angle_index].emplace(std::make_tuple(xv, yv, zv));
                }
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
//            const auto &[_, __, ceta] = *bin.rbegin();
//            std::cout << "Max in bin: " << ceta << std::endl;
        }
        else
            ldata[i].dist = 0.f;
        i++;
    }
    return ldata;
}

void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata)
{
    if(ldata.empty()) return;

    const int lado = 800;
    cv::Mat laser_img(cv::Size(lado, lado), CV_8UC3);
    laser_img = cv::Scalar(255,255,255);
    float scale = 100;
    float x = ldata.front().dist * sin(ldata.front().angle) * scale + lado/2;
    float y = lado - ldata.front().dist * cos(ldata.front().angle) * scale;
    cv::line(laser_img, cv::Point{lado/2,lado}, cv::Point(x,y), cv::Scalar(0,200,0));
    for(auto &&l : iter::sliding_window(ldata, 2))
    {
        int x1 = l[0].dist * sin(l[0].angle) * scale + lado/2;
        int y1 = 500 - l[0].dist * cos(l[0].angle) * scale;
        int x2 = l[1].dist * sin(l[1].angle) * scale + lado/2;
        int y2 = 500 - l[1].dist * cos(l[1].angle) * scale;
        cv::line(laser_img, cv::Point{x1,y1}, cv::Point(x2,y2), cv::Scalar(0,200,0));
    }
    x = ldata.back().dist * sin(ldata.back().angle) * scale + lado/2;
    y = lado - ldata.back().dist * cos(ldata.back().angle) * scale;
    cv::line(laser_img, cv::Point(x,y), cv::Point(lado/2,lado), cv::Scalar(0,200,0));

    cv::imshow("Laser", laser_img);
    cv::waitKey(2);
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
    qWarning() << __FUNCTION__ << "Not implemented";
    RoboCompLaser::TLaserData dummy;
    return dummy;
}

RoboCompLaser::LaserConfData SpecificWorker::Laser_getLaserConfData()
{
    qWarning() << __FUNCTION__ << "Not implemented";
    RoboCompLaser::LaserConfData dummy;
    return dummy;
}

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserData()
{
    std::lock_guard<std::mutex> lg(my_mutex);
    return ldata;
}




/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

