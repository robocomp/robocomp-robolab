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
#include "specificworker.h"


/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
    //QLoggingCategory::setFilterRules("*.debug=false\n");
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
    serial_left = params["serial_left"].value;
    serial_right = params["serial_right"].value;
    serial_center = params["serial_center"].value;
    display_depth = (params["display_depth"].value == "true") or (params["display_depth"].value == "True");
    display_rgb = (params["display_rgb"].value == "true") or (params["display_rgb"].value == "True");
    display_laser = (params["display_laser"].value == "true") or (params["display_laser"].value == "True");
    compressed = (params["compressed"].value == "true") or (params["compressed"].value == "True");
    publish = (params["publish"].value == "true") or (params["publish"].value == "True");
    consts.max_up_height = std::stof(params.at("max_up_height").value);
    consts.max_down_height = std::stof(params.at("max_down_height").value);

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
        compression_params_image.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params_image.push_back(75);
        compression_params_depth.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params_depth.push_back(75);

        int fps_depth = 30;
        int fps_color = 30;
        // center camera
        try
        {
            std::cout << "Opening center camera " << serial_center;
            cfg_center.enable_device(serial_center);
            cfg_center.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16, fps_depth);
            cfg_center.enable_stream(RS2_STREAM_COLOR, consts.width, consts.height, RS2_FORMAT_BGR8, fps_color);
            rs2::pipeline center_pipe;
            rs2::pipeline_profile profile_center = center_pipe.start(cfg_center);
            center_depth_intr = center_pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
            Eigen::Translation<float, 3> center_tr(0., 0.,  -0.020);
            Eigen::Matrix3f center_m;
            center_m = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX())
                       * Eigen::AngleAxisf(0.244346, Eigen::Vector3f::UnitY())  //60 degrees
                       * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ());
            Eigen::Transform<float, 3, Eigen::Affine> center_depth_extrinsics;;
            center_depth_extrinsics = center_tr;
            center_depth_extrinsics.rotate(center_m);
            cam_map[serial_center] = std::make_tuple(center_pipe, center_depth_intr, center_depth_extrinsics, rs2::frame(), rs2::points(), rs2::frame());
            print_camera_params(serial_center, profile_center);
            qInfo() << __FUNCTION__ << " center-camera started";
        }
        catch(const std::exception &e)
        { std::cout << e.what()<< " Serial number:" << serial_center << std::endl; std::terminate();}

        // right camera
//    try
//    {
//        cfg_right.enable_device(serial_right);
////        cfg_right.enable_stream(RS2_STREAM_DEPTH, consts.width, consts.height, RS2_FORMAT_Z16, fps_depth);
//        cfg_right.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16, fps_depth);
//        cfg_right.enable_stream(RS2_STREAM_COLOR, consts.width, consts.height, RS2_FORMAT_BGR8, fps_color);
//        rs2::pipeline right_pipe;
//        rs2::pipeline_profile profile_right = right_pipe.start(cfg_right);
//        right_depth_intr = right_pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
//        Eigen::Translation<float, 3> right_tr(0.0, -0.020, 0.0);
//        // Eigen::Translation<float, 3> right_tr(0.0963, 0., 0.0578);
//        Eigen::Matrix3f right_m;
//        right_m = Eigen::AngleAxisf(M_PI/3, Eigen::Vector3f::UnitX())
//                  * Eigen::AngleAxisf(0.244346, Eigen::Vector3f::UnitY())  //60 degrees
//                  * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ());
//        Eigen::Transform<float, 3, Eigen::Affine> right_depth_extrinsics;;
//        right_depth_extrinsics = right_tr;
//        right_depth_extrinsics.rotate(right_m);
////        std::cout << "right transform " << right_depth_extrinsics.matrix() << std::endl;
//        cam_map[serial_right] = std::make_tuple(right_pipe, right_depth_intr, right_depth_extrinsics,  rs2::frame(), rs2::points(), rs2::frame());
////        print_camera_params(serial_right, profile_right);
//        qInfo() << __FUNCTION__ << " right-camera started";
//    }
//    catch(const std::exception &e)
//    { std::cout << e.what()<< " Serial number:" << serial_right << std::endl; std::terminate();}

        // left camera
        try
        {
            std::cout << "Opening right camera " << serial_left;
            cfg_left.enable_device(serial_left);
            cfg_left.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16, fps_depth);
            cfg_left.enable_stream(RS2_STREAM_COLOR, consts.width, consts.height, RS2_FORMAT_BGR8, fps_color);
            rs2::pipeline left_pipe;
            rs2::pipeline_profile profile_left = left_pipe.start(cfg_left);
            left_depth_intr = left_pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
            Eigen::Translation<float, 3> left_tr(0, 0.020, 0.0);
            // Eigen::Translation<float, 3> left_tr(-0.0963, 0., 0.0578);
            Eigen::Matrix3f left_m;
            left_m = Eigen::AngleAxisf(-M_PI/3., Eigen::Vector3f::UnitX())
                     * Eigen::AngleAxisf(0.244346, Eigen::Vector3f::UnitY())  //60 degrees
                     * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ());
            Eigen::Transform<float, 3, Eigen::Affine> left_depth_extrinsics;;
            left_depth_extrinsics = left_tr;
            left_depth_extrinsics.rotate(left_m);
            cam_map[serial_left] = std::make_tuple(left_pipe, left_depth_intr, left_depth_extrinsics, rs2::frame(), rs2::points(), rs2::frame());
//        print_camera_params(serial_left, profile_left);
            qInfo() << __FUNCTION__ << " left-camera started";
        }
        catch(const std::exception &e)
        { std::cout << e.what()<< " Serial number:" << serial_left << std::endl; std::terminate();}

        // Filters
        rs2_set_option(dec_filter, RS2_OPTION_FILTER_MAGNITUDE, 8, error);
        filters.emplace_back("Decimate", dec_filter);
        filters.emplace_back("Spatial", spat_filter);
        filters.emplace_back("Temporal", temp_filter);
        filters.emplace_back("HFilling", holef_filter);

        //compression params
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params.push_back(50);

        timer.start(33);
	}

    //PATH LOADING
}

void SpecificWorker::compute()
{
    auto &cam_map_extended = read_and_filter(cam_map);   // USE OPTIONAL
    auto ldata_local = compute_laser(cam_map_extended);
    auto virtual_frame = mosaic(cam_map_extended);

    if(display_rgb)
    {
        cv::imshow("Dual Camera", virtual_frame),
        cv::waitKey(1);
    }

    RoboCompCameraRGBDSimple::TImage im;
    im.width = virtual_frame.cols; im.height = virtual_frame.rows;
    if(compressed)
    {
        cv::imencode(".jpg", virtual_frame, buffer, compression_params);
        im.image = buffer;
        im.compressed = true;
     }
     else
     {
        int img_size = virtual_frame.rows*virtual_frame.cols*3;  // TODO: cambiar
        im.image.resize(img_size);
        memcpy(&im.image[0], virtual_frame.data, img_size);
        im.compressed = false;
     }
    RoboCompCameraRGBDSimple::TDepth depth;
//    depth.width = virtual_frame.cols; depth.height = virtual_frame.rows;  // CAMBIAR !!!!!!!!!!!!!!1
//    int depth_size = virtual_frame.rows*virtual_frame.cols*sizeof(float);  // CAMBIAR !!!!!!!!!!!!!!1
//
//    depth.depth.resize(depth_size);
//    memcpy(&depth.depth[0], virtual_frame_depth.data, depth_size);
//    depth.compressed = false;

    if(publish)
    {
        this->camerargbdsimplepub_pubproxy->pushRGBD(im, depth);
        this->laserpub_pubproxy->pushLaserData(ldata_local);
    }
    // copy to output buffers
    buffer_image.put(std::move(im));
    buffer_depth.put(std::move(depth));
    fps.print("FPS: ");
}

//////////////////////////////////////////////////////////////////////////////////////////
SpecificWorker::Camera_Map& SpecificWorker::read_and_filter(Camera_Map &cam_map)
{
    for (auto &[key, value] : cam_map)
    {
        //if(key != serial_center and key != serial_right) continue;
        auto &[my_pipe, intr, extr, depth_frame, points, color_frame] = value;
        rs2::frameset data = my_pipe.wait_for_frames();
        depth_frame = data.get_depth_frame(); // Find and colorize the depth dat
        color_frame = data.get_color_frame(); // Find and colorize the depth dat

        for (auto &&filter : filters)
            depth_frame = filter.filter.process(depth_frame);

//      rgb_list[i] = data.get_color_frame(); // Find the color data
        //rs2::pointcloud pointcloud;
        //pointcloud.map_to(color_frame);
        //points = pointcloud.calculate(depth_frame);
    }
    return cam_map;
}

////////////////// MOSAIC  WITHOUT PROCESSING //////////////////////////////////////////
cv::Mat SpecificWorker::mosaic(const Camera_Map &cam_map)
{
    // virtual frame
    cv::Mat frame_virtual = cv::Mat::zeros(consts.width, consts.height*3, CV_8UC3);
    cv::Mat winLeftR = frame_virtual(cv::Rect(0,0,consts.height, consts.width));
    cv::Mat winRightR = frame_virtual(cv::Rect(consts.height,0,consts.height,consts.width));

    cv::Mat frame_virtual_depth = cv::Mat::zeros(consts.width, consts.height*2, CV_8UC3);
    cv::Mat winLeftR_depth = frame_virtual_depth(cv::Rect(0, 0, consts.height, consts.width));
    cv::Mat winRightR_depth = frame_virtual_depth(cv::Rect(consts.height, 0, consts.height,consts.width));

    const auto &[pipeL, intrL, extrL, depth_frameL, pointsL, color_frameL] = cam_map.at(serial_left);
    cv::Mat imgLeft(consts.height, consts.width, CV_8UC3, (char *) color_frameL.get_data());
    cv::rotate(imgLeft, winLeftR, cv::ROTATE_90_CLOCKWISE);
    cv::Mat depthLeft(consts.height, consts.width, CV_32FC1, (float *) depth_frameL.get_data());
    cv::rotate(depthLeft, winLeftR_depth, cv::ROTATE_90_CLOCKWISE);

    const auto &[pipeR, intrR, extrR, depth_frameR, pointsR, color_frameR] = cam_map.at(serial_center);
    cv::Mat imgRight(consts.height, consts.width, CV_8UC3, (char *) color_frameR.get_data());
    cv::rotate(imgRight, winRightR, cv::ROTATE_90_CLOCKWISE);
    cv::Mat depthRight(consts.height, consts.width, CV_32FC1, (float *) depth_frameR.get_data());
    cv::rotate(depthRight, winRightR_depth, cv::ROTATE_90_CLOCKWISE);

    return frame_virtual;
}

/////////////////////////////////////////////////////////////////////
///////////////////// DISPLAY //////////////////////////////////////
////////////////////////////////////////////////////////////////////

void SpecificWorker::show_depth_images(Camera_Map &cam_map)
{
    std::map<std::string,cv::Mat> image_stack;
    int w=0, h=0;
    for (auto &[key, value] : cam_map)
    {
        auto &[pipe, intr, extr, depth_frame, points, color_frame] = value;
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
    if(not cam_map.empty())
    {
        cv::Mat frame_final(cv::Size(w * image_stack.size(), h), CV_8UC3);
        image_stack[serial_left].copyTo(frame_final(cv::Rect(0,0,w,h)));
        image_stack[serial_center].copyTo(frame_final(cv::Rect(w,0,w,h)));
        image_stack[serial_right].copyTo(frame_final(cv::Rect(2*w,0,w,h)));
        //        for(auto &&[i, img] : iter::enumerate(image_stack))
        //            img.copyTo(frame_final(cv::Rect(i*w,0,w,h)));
        //cv::imshow("Depth mosaic", frame_final);
        //cv::waitKey(1);
    }
}

void SpecificWorker::print_camera_params(const std::string &serial, const rs2::pipeline_profile &profile)
{
    float center_fov[2]; // X, Y fov
    const auto &[pipe, intr, extr, depth_frame, points, color_frame] = cam_map.at(serial);
    rs2_fov(&intr, center_fov);
//    std::cout << "Camera " << serial << " started" << std::endl;
//    std::cout << "  width: " << center_depth_intr.width << std::endl;
//    std::cout << "  height: " << center_depth_intr.height << std::endl;
//    std::cout << "  image center x: " << center_depth_intr.ppx << std::endl;
//    std::cout << "  image center y: " << center_depth_intr.ppy << std::endl;
//    std::cout << "  focal x: " << center_depth_intr.fx << std::endl;
//    std::cout << "  focal y: " << center_depth_intr.fy << std::endl;
//    std::cout << "  horizontal angle: " << center_fov[0] << std::endl;
//    std::cout << "  vertical angle: " << center_fov[1] << std::endl;
//    std::cout << "  extrinsics: " << extr.matrix() << std::endl;
}

RoboCompLaser::TLaserData SpecificWorker::compute_laser(const Camera_Map &cam_map_extended)
{
    using Point = std::tuple<float, float, float>;
    auto cmp = [](Point a, Point b)
    {
        auto &[ax, ay, az] = a;
        auto &[bx, by, bz] = b;
        return (ax * ax + ay * ay + az * az) < (bx * bx + by * by + bz * bz);
    };
    std::vector<std::set<Point, decltype(cmp) >> hor_bins(consts.MAX_LASER_BINS);   // lambda to sort elements on insertion

    for( const auto &[key, value] : cam_map_extended)
    {
        const auto &[pipe, intrin, extrin, depth_frame, points, color_frame] = value;
        if(points.size() == 0) continue;

        const rs2::vertex *vertices = points.get_vertices();

        float FLOOR_DISTANCE_MINUS_OFFSET =  extrin.translation().x() * 0.9;  // X+ axis points downwards since the camera is rotated
        for (size_t i = 0; i < points.size(); i++)
        {
            if((vertices[i].z >= 0.99 ) and (vertices[i].z <= 15)) //and vertices[i].z <=3
            {
                auto to_point = extrin * Eigen::Vector3f{vertices[i].x, vertices[i].y, vertices[i].z};

                const float &xv = to_point[0]; const float &yv = to_point[1]; const float &zv = to_point[2];
                //if (xv < RIG_ELEVATION_FROM_FLOOR * 0.9)
                if( xv < consts.max_down_height and xv > consts.max_up_height)  // central band - positive x downwards
                {
                    float hor_angle = atan2(yv, zv);
                    // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
                    int angle_index = (int) ((consts.MAX_LASER_BINS / consts.TOTAL_HOR_ANGLE) * hor_angle + (consts.MAX_LASER_BINS / 2.f));
                    if (angle_index >= consts.MAX_LASER_BINS or angle_index < 0) continue;
                    hor_bins[angle_index].emplace(std::make_tuple(xv, yv, zv)); //necesario para laser merge en agent control
                }
            }
        }
    }
    RoboCompLaser::TLaserData ldata(consts.MAX_LASER_BINS);
    uint i = 0;
    for (auto &bin : hor_bins)
    {

        // int posicion= hor_bins - i;
        ldata[(hor_bins.size()-1) - i].angle = (i - consts.MAX_LASER_BINS / 2.f) * ( consts.TOTAL_HOR_ANGLE / consts.MAX_LASER_BINS );
        if (bin.size() > 0)
        {
            const auto &[X, Y, Z] = *bin.cbegin();

            ldata[i].dist = sqrt(X * X + Y * Y + Z * Z)*1000;
            ldata[i].dist;

        }
        else
        {
            if(i>0)
                ldata[i].dist = ldata[i - 1].dist;  // link to the adjacent
            else
                ldata[i].dist = 10000;
        }
//        qInfo() << "DISTANCES" << ldata[i].dist;
        i++;
    }
    return ldata;
}

void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata)
{
    if(ldata.empty()) return;

    const int lado = 800;
    const int semilado = lado/2;
    const int yoffset = 600;
    cv::Mat laser_img(cv::Size(lado, lado), CV_8UC3);
    laser_img = cv::Scalar(255,255,255);
    float scale = 0.1;
    cv::circle(laser_img, cv::Point(semilado,yoffset), 10, cv::Scalar(200,100,0), cv::FILLED, 8,0);
    float x = ldata.front().dist * sin(ldata.front().angle) * scale + lado/2;
    float y = yoffset - ldata.front().dist * cos(ldata.front().angle) * scale;
    cv::line(laser_img, cv::Point{semilado,yoffset}, cv::Point(x,y), cv::Scalar(0,200,0));
    for(auto &&l : iter::sliding_window(ldata, 2))
    {
        int x1 = l[0].dist * sin(l[0].angle) * scale + semilado;
        int y1 = yoffset - l[0].dist * cos(l[0].angle) * scale;
        int x2 = l[1].dist * sin(l[1].angle) * scale + semilado;
        int y2 = yoffset - l[1].dist * cos(l[1].angle) * scale;
        cv::line(laser_img, cv::Point{x1,y1}, cv::Point(x2,y2), cv::Scalar(0,200,0));
    }
    x = ldata.back().dist * sin(ldata.back().angle) * scale + semilado;
    y = yoffset - ldata.back().dist * cos(ldata.back().angle) * scale;
    cv::line(laser_img, cv::Point(x,y), cv::Point(semilado,yoffset), cv::Scalar(0,200,0));

    //cv::imshow("Laser", laser_img);
    //cv::waitKey(2);
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

/**************************************/
// From the RoboCompCameraRGBDSimplePub you can publish calling this methods:
// this->camerargbdsimplepub_pubproxy->pushRGBD(...)

/**************************************/
// From the RoboCompLaserPub you can publish calling this methods:
// this->laserpub_pubproxy->pushLaserData(...)

RoboCompCameraRGBDSimple::TRGBD SpecificWorker::CameraRGBDSimple_getAll(std::string camera)
{
    RoboCompCameraRGBDSimple::TRGBD rgbd;
    rgbd.image = buffer_image.get();
    return rgbd;
}

RoboCompCameraRGBDSimple::TDepth SpecificWorker::CameraRGBDSimple_getDepth(std::string camera)
{
    return RoboCompCameraRGBDSimple::TDepth();
}

RoboCompCameraRGBDSimple::TImage SpecificWorker::CameraRGBDSimple_getImage(std::string camera)
{
    return buffer_image.get();
}

RoboCompCameraRGBDSimple::TPoints SpecificWorker::CameraRGBDSimple_getPoints(std::string camera)
{
    return RoboCompCameraRGBDSimple::TPoints();
}

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState)
{
    return RoboCompLaser::TLaserData();
}

RoboCompLaser::LaserConfData SpecificWorker::Laser_getLaserConfData()
{
    return RoboCompLaser::LaserConfData();
}

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserData()
{
    return RoboCompLaser::TLaserData();

}
/**************************************/
// From the RoboCompCameraRGBDSimplePub you can publish calling this methods:
// this->camerargbdsimplepub_pubproxy->pushRGBD(...)

/**************************************/
// From the RoboCompLaserPub you can publish calling this methods:
// this->laserpub_pubproxy->pushLaserData(...)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::Point3D
// RoboCompCameraRGBDSimple::TPoints
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

