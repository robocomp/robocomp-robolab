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
    std::cout << "display_rgb " << display_depth << " display_depth " << display_rgb << std::endl;
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
        profile = pipe.start(cfg);

        // Each depth camera might have different units for depth pixels, so we get it here
        // Using the pipeline's profile, we can retrieve the device that the pipeline uses
        depth_scale = get_depth_scale(profile.get_device());

        // Create a rs2::align object.
        align_to_rgb = pipe.get_active_profile().get_stream(RS2_STREAM_COLOR).stream_type();
        align = std::make_unique<rs2::align>(align_to_rgb);

        // Define a variable for controlling the distance to clip
        float depth_clipping_distance = 10000.f;

        cam_intr = pipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
        depth_intr = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();

        // filters.emplace_back("Decimate", dec_filter);
        // filters.emplace_back(disparity_filter_name, depth_to_disparity);
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

    //Get processed aligned frame
    auto processed = align->process(frameset);

    // Trying to get both rgb and aligned depth frames
    rgb_frame = processed.first(align_to_rgb);
    depth_frame = processed.get_depth_frame();

    //If one of them is unavailable, continue iteration
    if (not depth_frame or not rgb_frame)
        return;

    for (auto &&filter : filters)
        if (filter.is_enabled)
            depth_frame = filter.filter.process(depth_frame);

    rgbd = create_trgbd();

    if (display_rgb)
    {
        cv::Mat frame(cv::Size(cam_intr.width, cam_intr.height), CV_8UC3, (void*)rgb_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::imshow("RGB image", frame);
        cv::waitKey(1);
    }
    if (display_depth)
    {
        rs2::frame depth_color = depth_frame.apply_filter(color_map);

        // Query frame size (width and height)
        const int w = depth_frame.as<rs2::video_frame>().get_width();
        const int h = depth_frame.as<rs2::video_frame>().get_height();

        cv::Mat frame_depth(cv::Size(w, h), CV_8UC3, (void*)depth_color.get_data(), cv::Mat::AUTO_STEP);
        cv::imshow("DEPTH image", frame_depth);
        cv::waitKey(1);
    }

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

RoboCompCameraRGBDSimple::TRGBD SpecificWorker::create_trgbd()
{
    RoboCompCameraRGBDSimple::TRGBD rgbd;
    auto rgb = rgb_frame.as<rs2::video_frame>();
    int width = rgb.get_width();
    int height = rgb.get_height();
    int rgb_bpp = rgb.get_bytes_per_pixel();
    rgbd.image.image.resize(width*height*rgb_bpp);
    rgbd.image.depth = rgb_bpp;
    rgbd.image.width = width;
    rgbd.image.height = height;
    rgbd.image.cameraID = 0;
    rgbd.image.focalx = cam_intr.fx;
    rgbd.image.focaly = cam_intr.fy;
    rgbd.image.alivetime = 0;

    rgbd.depth.depth.resize(width*height*4);
    rgbd.depth.width = width;
    rgbd.depth.height = height;
    rgbd.depth.cameraID = 0;
    rgbd.depth.focalx = depth_intr.fx;
    rgbd.depth.focaly = depth_intr.fy;
    rgbd.depth.alivetime = 0;

    const uint16_t* p_depth_frame = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
    uint8_t* p_rgb_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(rgb_frame.get_data()));
    int k=0;
    for (int y = 0; y < height; y++)
    {
        auto depth_pixel_index = y * width;
        for (int x = 0; x < width; x++, ++depth_pixel_index)
        {
            // Get the depth value of the current pixel
            const float real_depth = depth_scale * p_depth_frame[depth_pixel_index];
            const std::uint8_t *d = reinterpret_cast<uint8_t const *>(&real_depth);
            rgbd.depth.depth[k] = d[0];
            rgbd.depth.depth[k+1] = d[1];
            rgbd.depth.depth[k+2] = d[2];
            rgbd.depth.depth[k+3] = d[3];
            k += 4;
            // Calculate the offset in rgb frame's buffer to current pixel
            auto offset = depth_pixel_index * rgb_bpp;
            rgbd.image.image[offset] = p_rgb_frame[offset];
            rgbd.image.image[offset+1] = p_rgb_frame[offset+1];
            rgbd.image.image[offset+2] = p_rgb_frame[offset+2];
        }
    }
    const std::lock_guard<std::mutex> lg(swap_mutex);
    return rgbd;
}
//////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}
////////////////////////////////////////////////////////////////////////////
RoboCompCameraRGBDSimple::TRGBD SpecificWorker::CameraRGBDSimple_getAll(std::string camera)
{
    const std::lock_guard<std::mutex> lg(swap_mutex);
    return rgbd;
}

RoboCompCameraRGBDSimple::TDepth SpecificWorker::CameraRGBDSimple_getDepth(std::string camera)
{

}

RoboCompCameraRGBDSimple::TImage SpecificWorker::CameraRGBDSimple_getImage(std::string camera)
{

}

/**************************************/
// From the RoboCompCameraRGBDSimplePub you can publish calling this methods:
// this->camerargbdsimplepub_pubproxy->pushRGBD(...)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

