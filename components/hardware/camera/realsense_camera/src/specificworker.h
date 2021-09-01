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
     RealSense D435 stereo camera RGBD server
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <librealsense2/rs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cppitertools/enumerate.hpp>
#include <opencv2/opencv.hpp>

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

public slots:
	void compute();
	int startup_check();
	void initialize(int period);

private:
	bool startup_check_flag;

	// camera
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    std::string serial;
    bool display_rgb = false;
    bool display_depth = false;
    rs2::pipeline  pipeline;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    rs2_intrinsics cam_intr, depth_intr;

    // filters
    struct filter_options
    {
        public:
            std::string filter_name;                                   //Friendly name of the filter
            rs2::filter &filter;                                       //The filter in use
            std::atomic_bool is_enabled;                               //A boolean controlled by the user that determines whether to apply the filter or not

            filter_options( const std::string name, rs2::filter &flt) :
                            filter_name(name),
                            filter(flt),
                            is_enabled(true)
            {
                const std::array<rs2_option, 5> possible_filter_options =
                {
                        RS2_OPTION_FILTER_MAGNITUDE,
                        RS2_OPTION_FILTER_SMOOTH_ALPHA,
                        RS2_OPTION_MIN_DISTANCE,
                        RS2_OPTION_MAX_DISTANCE,
                        RS2_OPTION_FILTER_SMOOTH_DELTA
                };
            }
            filter_options(filter_options&& other) :
                    filter_name(std::move(other.filter_name)),
                    filter(other.filter),
                    is_enabled(other.is_enabled.load())
            { }
    };

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // filters
    std::vector<filter_options> filters;
    rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;
    rs2::hole_filling_filter holef_filter;
    rs2::disparity_transform depth_to_disparity;
    rs2::disparity_transform disparity_to_depth;
    const std::string disparity_filter_name = "Disparity";

    rs2::pipeline  pipe;
    rs2::frame rgb_frame, depth_frame;

    std::unique_ptr<rs2::align> align;
    rs2::pipeline_profile profile;
    rs2_stream align_to_rgb;
    float depth_scale;

    float get_depth_scale(rs2::device dev);
    RoboCompCameraRGBDSimple::TRGBD create_trgbd();
    mutable std::mutex swap_mutex;
    RoboCompCameraRGBDSimple::TRGBD rgbd;
};

#endif
