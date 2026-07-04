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

/*
 * # RicohOmni

Installation of Ricoh drivers

Goto: https://codetricity.github.io/theta-linux/software/  and choose the

    Using gstreamer and OpenCV without v4l2loopback

option, and install https://github.com/nickel110/gstthetauvc

Install de NVidia GStreamer plugin with: https://gist.github.com/corenel/a615b6f7eb5b5425aa49343a7b409200

Add to .bashrc and source:
    export GST_PLUGIN_PATH=/home/robocomp/software/gstthetauvc/thetauvc

 */

/**
	\brief
	@author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

// HIBERNATION DESACTIVADA para mantener máxima velocidad continua
// Si se activa, reduce el período cuando no hay peticiones
// #define HIBERNATION_ENABLED

#include <genericworker.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <fps/fps.h>
#include <doublebuffer/DoubleBuffer.h>
#include <thread>
#include <chrono>
#include <memory>

using namespace std::chrono;

// Zero-copy DDS media-plane publisher (Image360), gated by config. FastDDS headers
// stay behind this forward declaration (PIMPL in dds_publisher.cpp).
class Ricoh360DDSPublisher;

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    /**
     * \brief Constructor for SpecificWorker.
     * \param configLoader Configuration loader for the component.
     * \param tprx Tuple of proxies required for the component.
     * \param startup_check Indicates whether to perform startup checks.
     */
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);

	/**
     * \brief Destructor for SpecificWorker.
     */
	~SpecificWorker();

	RoboCompCamera360RGB::TImage Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight);

	std::string MediaPlaneDDS_getMediaDescriptor();

public slots:

	/**
	 * \brief Initializes the worker one time.
	 */
	void initialize();

	/**
	 * \brief Main compute loop of the worker.
	 */
	void compute();

	/**
	 * \brief Handles the emergency state loop.
	 */
	void emergency();

	/**
	 * \brief Restores the component from an emergency state.
	 */
	void restore();

    /**
     * \brief Performs startup checks for the component.
     * \return An integer representing the result of the checks.
     */
	int startup_check();

private:

	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;
	bool activated_camera = false;
	cv::Mat cv_frame;
	cv::VideoCapture capture;

	// Streaming paralelo a MediaMTX
	cv::VideoWriter stream_writer;
	std::string stream_pipeline;
	bool streaming_enabled = false;

	std::vector<int> compression_params;
	std::string pipeline;

	FPSCounter fps;
	std::atomic<std::chrono::high_resolution_clock::time_point> last_read;
	int MAX_INACTIVE_TIME = 5;  // secs after which the component is paused. It reactivates with a new reset

	struct PARAMS
	{
		std::string device = "/dev/video0";
		bool display = false;
		bool orin = false;
		bool use_gpu = true;
		bool compressed = false;
		bool simulator = false;
		bool streaming = true;      // Streaming paralelo a MediaMTX
		bool stream_webrtc = false; // true=WHIP/WebRTC directo, false=RTMP a MediaMTX
		std::string stream_host = "localhost";
		int stream_port = 8554;
		std::string stream_path = "theta";
		int time_offset;
	};
	PARAMS pars;
	int MAX_WIDTH, MAX_HEIGHT, DEPTH;
	RoboCompCamera360RGB::TImage image;

	DoubleBuffer<cv::Mat, cv::Mat> buffer_image;

	long long capture_time;
	// track image period
	void self_adjust_period(int new_period);

	// Optional zero-copy DDS 360 media plane (null unless PublishDDS is enabled).
	bool publish_dds = false;
	std::unique_ptr<Ricoh360DDSPublisher> dds_publisher;

signals:
	//void customSignal();
};

#endif
