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

	const int w = 640;
    const int h = 480;
    try
    {
        cfg_center.enable_device(serial_center);
        cfg_center.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, 15);
        cfg_center.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, 15);
        rs2::pipeline center_pipe;
        rs2::pipeline_profile profile_center = center_pipe.start(cfg_center);
        center_depth_intr = center_pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
        Eigen::Translation<float, 3> center_tr(0.f, 0, 0.1004);
        // Eigen::Translation<float, 3> center_tr(0.f, 0, 0.100);
        Eigen::Matrix3f center_m;
        center_m = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX())
                 * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY())
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
    try
    {
        cfg_right.enable_device(serial_right);
        cfg_right.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, 15);
        cfg_right.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, 15);
        rs2::pipeline right_pipe;
        rs2::pipeline_profile profile_right = right_pipe.start(cfg_right);
        right_depth_intr = right_pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
        Eigen::Translation<float, 3> right_tr(0.0966, 0., 0.0276);
        // Eigen::Translation<float, 3> right_tr(0.0963, 0., 0.0578);
        Eigen::Matrix3f right_m;
        right_m = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX())
                * Eigen::AngleAxisf(2.*M_PI/5., Eigen::Vector3f::UnitY())  //60 degrees
                * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ());
        Eigen::Transform<float, 3, Eigen::Affine> right_depth_extrinsics;;
        right_depth_extrinsics = right_tr;
        right_depth_extrinsics.rotate(right_m);
        std::cout << "right transform " << right_depth_extrinsics.matrix() << std::endl;
        cam_map[serial_right] = std::make_tuple(right_pipe, right_depth_intr, right_depth_extrinsics,  rs2::frame(), rs2::points(), rs2::frame());
        print_camera_params(serial_right, profile_right);
        qInfo() << __FUNCTION__ << " right-camera started";
    }
    catch(const std::exception &e)
    { std::cout << e.what()<< " Serial number:" << serial_right << std::endl; std::terminate();}

    // left camera
    try
    {
        cfg_left.enable_device(serial_left);
        cfg_left.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, 15);
        cfg_left.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, 15);
        rs2::pipeline left_pipe;
        rs2::pipeline_profile profile_left = left_pipe.start(cfg_left);
        left_depth_intr = left_pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
        Eigen::Translation<float, 3> left_tr(-0.0966, 0., 0.0276);
        // Eigen::Translation<float, 3> left_tr(-0.0963, 0., 0.0578);
        Eigen::Matrix3f left_m;
        left_m = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX())
                * Eigen::AngleAxisf(-2.*M_PI/5., Eigen::Vector3f::UnitY())
                * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ());
        Eigen::Transform<float, 3, Eigen::Affine> left_depth_extrinsics;;
        left_depth_extrinsics = left_tr;
        left_depth_extrinsics.rotate(left_m);
        cam_map[serial_left] = std::make_tuple(left_pipe, left_depth_intr, left_depth_extrinsics, rs2::frame(), rs2::points(), rs2::frame());
        print_camera_params(serial_left, profile_left);
        qInfo() << __FUNCTION__ << " left-camera started";
    }
    catch(const std::exception &e)
    { std::cout << e.what()<< " Serial number:" << serial_left << std::endl; std::terminate();}

    // Filters
    rs2_set_option(dec_filter, RS2_OPTION_FILTER_MAGNITUDE, 4, error);
    // filters.emplace_back("Decimate", dec_filter);
    filters.emplace_back("Spatial", spat_filter);
    filters.emplace_back("Temporal", temp_filter);
    filters.emplace_back("HFilling", holef_filter);

    this->Period = 50;
	if(this->startup_check_flag)
		this->startup_check();
	else
		timer.start(Period);
}

void SpecificWorker::compute()
{
    auto &cam_map_extended = read_and_filter(cam_map);   // USE OPTIONAL
    auto ldata_local = compute_laser(cam_map_extended);
    auto &&[virtual_frame] = mosaic(cam_map_extended);

    if(display_depth)
        show_depth_images(cam_map_extended);

    if(display_rgb)
    {
        //cv::flip(virtual_frame, virtual_frame, 0);
        cv::imshow("Virtual", virtual_frame);
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

        auto &[my_pipe, intr, extr, depth_frame, points, color_frame] = value;
        rs2::frameset data = my_pipe.wait_for_frames();
        depth_frame = data.get_depth_frame(); // Find and colorize the depth dat
        color_frame = data.get_color_frame(); // Find and colorize the depth dat

        for (auto &&filter : filters)
            depth_frame = filter.filter.process(depth_frame);

        // rgb_list[i] = data.get_color_frame(); // Find the color data
        rs2::pointcloud pointcloud;
        pointcloud.map_to(color_frame);
        points = pointcloud.calculate(depth_frame);
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
        const auto &[pipe, intrin, extrin, depth_frame, points, color_frame] = value;
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
                if (yv < RIG_ELEVATION_FROM_FLOOR * 0.9)
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

////////////////// MOSAIC ///////////////////////////////////////////////////
std::tuple<cv::Mat> SpecificWorker::mosaic(const Camera_Map &cam_map)
{
    // virtual frame
    cv::Mat frame_virtual = cv::Mat::zeros(cv::Size(640*3, 480*1.5), CV_8UC3);

    cv::Mat sphere = cv::Mat::zeros(cv::Size(800, 600), CV_8UC3);
    float sphere_radius =  sphere.rows/2.;
    float sphere_cols = sphere.cols;
    float sphere_rows = sphere.rows;



    float center_virtual_cols = frame_virtual.cols / 2.0;
    float center_virtual_rows = frame_virtual.rows / 2.0;
    float frame_virtual_lfocalx = 390;
    float frame_virtual_rfocalx = 390;


    //if (left_cam_intr.width == left_depth_intr.width and left_cam_intr.height == left_depth_intr.height)
    // check size requirements

    for(const auto &[key, cam] : cam_map)
    {
        const auto &[pipe, intr, extr, depth_frame, points, color_frame] = cam;
        rs2::video_frame video_frame(color_frame);
        const rs2::vertex *vertices = points.get_vertices();
        auto tex_coords = points.get_texture_coordinates(); // and texture coordinates, u v coor of rgb image
        for (size_t i = 0; i < points.size(); i++)
        {
            if (vertices[i].z >= 0.1)
            {
                // transform to virtual camera CS at center of both cameras.
                auto to_point = extr * Eigen::Vector3f{vertices[i].x, vertices[i].y, vertices[i].z};
                const float &xv = to_point[0]; const float &yv = to_point[1]; const float &zv = to_point[2];

                float cX = atan2(xv, zv) * sphere_cols/M_PI + sphere_cols/2.;
                float cY = atan2(yv, zv) * sphere_rows/M_PI + sphere_rows/2.;
                if (cX < 0 or cX >= sphere_cols or cY < 0 or cY >= sphere_rows) continue;
                int k = tex_coords[i].v * video_frame.get_height();
                int l = tex_coords[i].u * video_frame.get_width();
                if (k < 0 or k >= video_frame.get_height() or l < 0 or l > video_frame.get_width()) continue;
                color(color_frame, sphere, cY, cX, k, l);
                // ////////////////////////////////
                // // project without sphere
                // float col_virtual = static_cast<float>((frame_virtual_lfocalx * xv / zv + center_virtual_cols));
                // float row_virtual = static_cast<float>((frame_virtual_lfocalx * yv / zv + center_virtual_rows));
                // if (col_virtual < 0 or col_virtual >= frame_virtual.cols or row_virtual < 0 or row_virtual >= frame_virtual.rows) continue;
                // int k = tex_coords[i].v * video_frame.get_height();
                // int l = tex_coords[i].u * video_frame.get_width();
                // if (k < 0 or k >= video_frame.get_height() or l < 0 or l > video_frame.get_width()) continue;
                // color(color_frame, frame_virtual, row_virtual, col_virtual, k, l);
                // ////////////////////////////////



                //cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(row_virtual, col_virtual);
//                auto ptr = (uint8_t *) video_frame.get_data();
//                auto stride = video_frame.get_stride_in_bytes();
//                color[0] = int(ptr[k * stride + (3 * l)]);
//                color[1] = int(ptr[k * stride + (3 * l) + 1]);
//                color[2] = int(ptr[k * stride + (3 * l) + 2]);

            }
        }   
    }

    cv::imshow("Sphere", sphere);
    cv::waitKey(1);


   for(int y=0; y<frame_virtual.rows; y++)
       for(int x=0; x<frame_virtual.cols; x++)
       {
            float xv = x - center_virtual_cols;
            float yv = y - center_virtual_rows;
            float cX = atan2(xv, frame_virtual_lfocalx) * sphere_cols/M_PI + sphere_cols/2.;
            float cY = atan2(yv, frame_virtual_lfocalx) * sphere_rows/M_PI + sphere_rows/2.;
            if (cX < 0 or cX >= sphere_cols or cY < 0 or cY > sphere_rows) continue;
            frame_virtual.at<cv::Vec3b>(y, x) = sphere.at<cv::Vec3b>(rint(cY), rint(cX));
       }


    // cv::resize(frame_virtual, frame_virtual, cv::Size(), 0.5, 0.5);

    // Image postprocessing
//    int TotalNumberOfPixels = frame_virtual.rows * frame_virtual.cols;
//    cv::Mat binaryImage;
//    cvtColor(frame_virtual, binaryImage, cv::COLOR_BGR2GRAY);
//    qInfo() << __FUNCTION__ << TotalNumberOfPixels - cv::countNonZero(binaryImage);

//    qInfo() << __FUNCTION__ << "one";
//    cv::Vec3b zero{0,0,0};
//    cv::Mat mask = cv::Mat::zeros(cv::Size(frame_virtual.cols, frame_virtual.rows), CV_8UC1);
//    for(int i=0; i<frame_virtual.rows; i++)
//        for(int j=0; j<frame_virtual.cols; j++)
//            if(frame_virtual.at<cv::Vec3b>(i,j) == zero)
//                mask.at<char>(i,j) = 1;
//
//    qInfo() << __FUNCTION__ << "two";
//    cv::inpaint(frame_virtual, mask, frame_virtual, 3, cv::INPAINT_TELEA);
//    qInfo() << __FUNCTION__ << "three";

//    cv::GaussianBlur(frame_virtual, frame_virtual, cv::Size(13, 13), 0, 0, 0);
//    float alpha = 4.0;
//    float beta = -1.0;
//    frame_virtual.convertTo(frame_virtual, -1, alpha, beta);

    return std::make_tuple(frame_virtual);
}

////////////////// MOSAIC  WITH AFFINE TRANSFORMATION //////////////////////////////////////////
// std::tuple<cv::Mat> SpecificWorker::mosaic(const Camera_Map &cam_map)
// {
//     // virtual frame
//     cv::Mat frame_virtual = cv::Mat::zeros(cv::Size(640*3, 480*1.5), CV_8UC3);

//     float center_virtual_cols = frame_virtual.cols / 2.0;
//     float center_virtual_rows = frame_virtual.rows / 2.0;
//     float frame_virtual_lfocalx = 390;
//     float frame_virtual_rfocalx = 390;

//     cv::Mat winLeft = frame_virtual(cv::Rect(0,120,640,480));
//     cv::Mat winCenter = frame_virtual(cv::Rect(640,120,640,480));
//     cv::Mat winRight = frame_virtual(cv::Rect(2*640,120,640,480));    


//     const auto &[pipeL, intrL, extrL, depth_frameL, pointsL, color_frameL] = cam_map.at(serial_left);
//     cv::Mat imgLeft(480, 640, CV_8UC3, (char *) color_frameL.get_data());
//     const auto &[pipeC, intrC, extrC, depth_frameC, pointsC, color_frameC] = cam_map.at(serial_center);
//     cv::Mat imgCenter(480, 640, CV_8UC3, (char *) color_frameC.get_data());
//     const auto &[pipeR, intrR, extrR, depth_frameR, pointsR, color_frameR] = cam_map.at(serial_right);
//     cv::Mat imgRight(480, 640, CV_8UC3, (char *) color_frameR.get_data());


//     imgCenter.copyTo(winCenter);
//     addImageToFrame(pointsL, extrC.inverse()*extrL, 120, 640, imgLeft, winLeft);
//     addImageToFrame(pointsR, extrC.inverse()*extrR, 0, -640, imgRight, winRight);
//     return std::make_tuple(frame_virtual);

// }

void SpecificWorker::addImageToFrame(rs2::points points, Eigen::Transform<float, 3, Eigen::Affine> extr, uint iniColumn, int columnShift, cv::Mat image, cv::Mat & frameWin)
{
    float frame_virtual_lfocalx = 390;
    float frame_virtual_rfocalx = 390;

    std::vector<cv::Point2d> imagePoints, framePoints;

    const rs2::vertex * vertices = points.get_vertices();
    auto tex_coords = points.get_texture_coordinates(); // and texture coordinates, u v coor of rgb image
    for (uint y = 0; y <120; y++)
        for (uint x = iniColumn; x <iniColumn + 40; x++)
        {
            uint i = y*160 + x;
            if (vertices[i].z >= 0.1)
            {
                // transform to virtual camera CS at center of both cameras.
                auto to_point = extr * Eigen::Vector3f{vertices[i].x, vertices[i].y, vertices[i].z};
                const float &xv = to_point[0]; const float &yv = to_point[1]; const float &zv = to_point[2];
                float dist = sqrt(xv*xv+yv*yv+zv*zv);
                if(dist < 0.4 or dist > 6.) continue;


                float col = static_cast<float>((frame_virtual_lfocalx * xv / zv + image.cols/2.));
                float row = static_cast<float>((frame_virtual_lfocalx * yv / zv + image.rows/2.));
                // if (col < 0 or col >= image.cols or row < 0 or row >= image.rows) continue;
                float k = tex_coords[i].v * image.rows;
                float l = tex_coords[i].u * image.cols;
                if (k < 0 or k >=  image.rows or l < 0 or l > image.cols) continue;
                imagePoints.push_back(cv::Point(l, k));
                framePoints.push_back(cv::Point(col+columnShift, row));

                // if(imagePoints.size()>20)
                //     break;

            }
        }   


    // cv::Mat affineT = getAffineTransform(imagePoints, framePoints);

    cv::Mat affineT;
    
    if(computeAffine(imagePoints, framePoints, affineT))
    {

        cv::Mat warped;

        cv::warpAffine(image, warped, affineT, cv::Size(640*2, 480*2));

    
        warped(cv::Rect(0,0,640,480)).copyTo(frameWin);
    }

}

// find affine transformation between two pointsets (use least square matching)
bool SpecificWorker::computeAffine(const std::vector<cv::Point2d> &srcPoints, const std::vector<cv::Point2d> &dstPoints, cv::Mat &transf)
{
    // sanity check
    if ((srcPoints.size() < 3) || (srcPoints.size() != dstPoints.size()))
        return false;

    // container for output
    transf.create(2, 3, CV_64F);

    // fill the matrices
    const int n = (int)srcPoints.size(), m = 3;
    cv::Mat A(n,m,CV_64F), xc(n,1,CV_64F), yc(n,1,CV_64F);
    for(int i=0; i<n; i++)
    {
        double x = srcPoints[i].x, y = srcPoints[i].y;
        double rowI[m] = {x, y, 1};
        cv::Mat(1,m,CV_64F,rowI).copyTo(A.row(i));
        xc.at<double>(i,0) = dstPoints[i].x;
        yc.at<double>(i,0) = dstPoints[i].y;
    }

    // solve linear equations (for x and for y)
    cv::Mat aTa, resX, resY;
    cv::mulTransposed(A, aTa, true);
    cv::solve(aTa, A.t()*xc, resX, cv::DECOMP_CHOLESKY);
    cv::solve(aTa, A.t()*yc, resY, cv::DECOMP_CHOLESKY);

    // store result
    memcpy(transf.ptr<double>(0), resX.data, m*sizeof(double));
    memcpy(transf.ptr<double>(1), resY.data, m*sizeof(double));

    return true;
}

////////////////// MOSAIC  WITHOUT PROCESSING //////////////////////////////////////////
// std::tuple<cv::Mat> SpecificWorker::mosaic(const Camera_Map &cam_map)
// {
//     // virtual frame
//     cv::Mat frame_virtual = cv::Mat::zeros(cv::Size(640*3, 480), CV_8UC3);

//     cv::Mat winLeft = frame_virtual(cv::Rect(0,0,640,480));
//     cv::Mat winCenter = frame_virtual(cv::Rect(640,0,640,480));
//     cv::Mat winRight = frame_virtual(cv::Rect(2*640,0,640,480));

//     const auto &[pipeL, intrL, extrL, depth_frameL, pointsL, color_frameL] = cam_map.at(serial_left);
//     cv::Mat imgLeft(480, 640, CV_8UC3, (char *) color_frameL.get_data());
//     imgLeft.copyTo(winLeft);
//     const auto &[pipeC, intrC, extrC, depth_frameC, pointsC, color_frameC] = cam_map.at(serial_center);
//     cv::Mat imgCenter(480, 640, CV_8UC3, (char *) color_frameC.get_data());
//     imgCenter.copyTo(winCenter);
//     const auto &[pipeR, intrR, extrR, depth_frameR, pointsR, color_frameR] = cam_map.at(serial_right);
//     cv::Mat imgRight(480, 640, CV_8UC3, (char *) color_frameR.get_data());
//     imgRight.copyTo(winRight);

//     return std::make_tuple(frame_virtual);

// }

void SpecificWorker::color(rs2::video_frame image, cv::Mat frame_v, float row_v, float col_v, int k, int l)
{
    auto ptr = (uint8_t *) image.get_data();
    auto stride = image.get_stride_in_bytes();

    cv::Vec3b &color = frame_v.at<cv::Vec3b>(floor(row_v), floor(col_v));
    color[0] = int(ptr[k * stride + (3 * l)]);
    color[1] = int(ptr[k * stride + (3 * l) + 1]);
    color[2] = int(ptr[k * stride + (3 * l) + 2]);
    color = frame_v.at<cv::Vec3b>(ceil(row_v), floor(col_v));
    color[0] = int(ptr[k * stride + (3 * l)]);
    color[1] = int(ptr[k * stride + (3 * l) + 1]);
    color[2] = int(ptr[k * stride + (3 * l) + 2]);
    color = frame_v.at<cv::Vec3b>(floor(row_v), ceil(col_v));
    color[0] = int(ptr[k * stride + (3 * l)]);
    color[1] = int(ptr[k * stride + (3 * l) + 1]);
    color[2] = int(ptr[k * stride + (3 * l) + 2]);
    color = frame_v.at<cv::Vec3b>(ceil(row_v), ceil(col_v));
    color[0] = int(ptr[k * stride + (3 * l)]);
    color[1] = int(ptr[k * stride + (3 * l) + 1]);
    color[2] = int(ptr[k * stride + (3 * l) + 2]);
}
///////////////////// DISPLAY ////////////////////////////////////77
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
        cv::imshow("Depth mosaic", frame_final);
        cv::waitKey(1);
    }
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
void SpecificWorker::print_camera_params(const std::string &serial, const rs2::pipeline_profile &profile)
{
    float center_fov[2]; // X, Y fov
    const auto &[pipe, intr, extr, depth_frame, points, color_frame] = cam_map.at(serial);
    rs2_fov(&intr, center_fov);
    std::cout << "Camera " << serial << " started" << std::endl;
    std::cout << "  width: " << center_depth_intr.width << std::endl;
    std::cout << "  height: " << center_depth_intr.height << std::endl;
    std::cout << "  image center x: " << center_depth_intr.ppx << std::endl;
    std::cout << "  image center y: " << center_depth_intr.ppy << std::endl;
    std::cout << "  focal x: " << center_depth_intr.fx << std::endl;
    std::cout << "  focal y: " << center_depth_intr.fy << std::endl;
    std::cout << "  horizontal angle: " << center_fov[0] << std::endl;
    std::cout << "  vertical angle: " << center_fov[1] << std::endl;
    std::cout << "  extrinsics: " << extr.matrix() << std::endl;
    for (auto p : profile.get_streams())
        std::cout << "  stream ID: " << p.unique_id() << " - Stream name: " << p.stream_name() << std::endl;
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

