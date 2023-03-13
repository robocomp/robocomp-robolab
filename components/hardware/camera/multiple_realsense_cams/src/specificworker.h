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
#include <innermodel/innermodel.h>
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include <doublebuffer/DoubleBuffer.h>
#include <iostream>
#include <string>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cppitertools/enumerate.hpp>
#include <opencv2/opencv.hpp>
#include <fps/fps.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/range.hpp>

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
    struct CONSTANTS
    {
        float RIG_ELEVATION_FROM_FLOOR = 1.0; // m
        int MAX_LASER_BINS = 180;  // 2 for each degrees
        float TOTAL_HOR_ANGLE = M_PI;  // D455 opens 87 x 58 degrees
        float max_up_height = 0;
        float max_down_height = 0;
        int width = 1280; //848 424 1280
        int height = 720; //480 240 720
    };
    CONSTANTS consts;

    using Camera_Map =  std::map<std::string,
            std::tuple<rs2::pipeline,
                    rs2_intrinsics,
                    Eigen::Transform<float, 3, Eigen::Affine>,
                    rs2::frame,
                    rs2::points,
                    rs2::frame>>;
    Camera_Map cam_map;

    // camera
    DoubleBuffer<std::vector<std::uint8_t>, std::vector<std::uint8_t>> virtual_camera_buffer;

    bool display_rgb = false;
    bool display_depth = false;
    bool display_laser = false;
    bool compressed = true;
    bool view = true;

    vector<int> compression_params_image;
    vector<int> compression_params_depth;

    rs2::config cfg_center, cfg_left, cfg_right;
    rs2_intrinsics center_cam_intr, left_cam_intr, right_cam_intr, center_depth_intr, left_depth_intr, right_depth_intr;
    rs2::context ctx;

   //Path
    std::vector<float> xpts, ypts;

	   // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    //mosaic
    cv::Mat virtual_frame;
    cv::Mat frame_virtual = cv::Mat::zeros(consts.width, consts.height*2, CV_8UC3);
    cv::Mat winLeftR = frame_virtual(cv::Rect(0,0,consts.height, consts.width));
    cv::Mat winCenterR = frame_virtual(cv::Rect(consts.height,0,consts.height, consts.width));
    //cv::Mat winRightR = frame_virtual(cv::Rect(2*consts.height,0,consts.height,consts.width));

    cv::Mat frame_virtual_depth = cv::Mat::zeros(consts.width, consts.height*2, CV_8UC3);
    cv::Mat winLeftR_depth = frame_virtual_depth(cv::Rect(0,0,consts.height, consts.width));
    cv::Mat winCenterR_depth = frame_virtual_depth(cv::Rect(consts.height,0,consts.height, consts.width));
    //cv::Mat winRightR_depth = frame_virtual_depth(cv::Rect(2*consts.height,0,consts.height,consts.width));

    // filters
    struct filter_options
    {
    public:
        std::string filter_name;                                   //Friendly name of the filter
        rs2::filter &filter;                                       //The filter in use
        std::atomic_bool is_enabled;                               //A boolean controlled by the user that determines whether to apply the filter or not

        filter_options( const std::string name, rs2::filter &flt) : filter_name(name),
                                                                    filter(flt),
                                                                    is_enabled(true)
        {};
        filter_options(filter_options&& other) : filter_name(std::move(other.filter_name)),
                                                 filter(other.filter),
                                                 is_enabled(other.is_enabled.load())
        {};
        const std::array<rs2_option, 5> possible_filter_options =
                {
                        RS2_OPTION_FILTER_MAGNITUDE,
                        RS2_OPTION_FILTER_SMOOTH_ALPHA,
                        RS2_OPTION_MIN_DISTANCE,
                        RS2_OPTION_MAX_DISTANCE,
                        RS2_OPTION_FILTER_SMOOTH_DELTA
                };
    };
    std::vector<filter_options> filters;
    rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;
    rs2::hole_filling_filter holef_filter;
    rs2::pipeline_profile profile;
    rs2::pointcloud pointcloud;
    rs2_error** error;

    // laser
    RoboCompLaser::TLaserData ldata;
    RoboCompLaser::TLaserData compute_laser(const Camera_Map &cam_map_extended);
    void draw_laser(const RoboCompLaser::TLaserData &ldata);

    std::string serial_center = "114122250668";
    std::string serial_left = "105322252279";
    std::string serial_right = "049522250171";

    // images
    void load_path(string filename);
    std::tuple<cv::Mat> mosaic(const Camera_Map &cam_map);
    Camera_Map& read_and_filter(Camera_Map &cam_map);
    void print_camera_params(const std::string &serial, const rs2::pipeline_profile &profile);
    void show_depth_images(Camera_Map &cam_map);
    std::mutex my_mutex;
    FPSCounter fps;
    vector<int> compression_params;
    vector<uchar> buffer;

    RoboCompCameraRGBDSimple::TImage im;
    RoboCompCameraRGBDSimple::TDepth dep;
};

#endif
