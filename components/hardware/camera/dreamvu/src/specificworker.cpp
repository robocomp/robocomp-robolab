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
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	PAL::Destroy();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }

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
		//Depth should be enable for occupancy map as a prerequisite
		bool isDepthEnabled = true;
		PAL::Internal::EnableDepth(isDepthEnabled);
		PAL::Internal::MinimiseCompute(false);
		int width, height;
		if(PAL::Init(width, height,-1) != PAL::SUCCESS) //Connect to the PAL camera
		{
			printf("Camera Init failed\n");
			std::terminate();
		}

		PAL::CameraProperties data;
		PAL::Acknowledgement ack = PAL::LoadProperties("Explorer/SavedPalProperties.txt", &data);
		if(ack != PAL::SUCCESS)
		{
			printf("Error Loading settings\n");
		}

        PAL::CameraProperties prop;
        unsigned int flag = PAL::MODE | PAL::FD | PAL::NR | PAL::FILTER_SPOTS | PAL::VERTICAL_FLIP ;
        prop.mode = PAL::Mode::FAST_DEPTH; // The other available option is PAL::Mode::HIGH_QUALITY_DEPTH
        //prop.mode = PAL::Mode::POINT_CLOUD_25D;
        prop.fd = 1;
        prop.nr = 1;
        prop.filter_spots = 1;
        prop.vertical_flip =0;
        PAL::SetCameraProperties(&prop, &flag);

//        p.resolution = {1280,448};
//        //5290x1819 3544x1218 1280x448 1120x384 640x224
//        p.color_space = PAL::RGB;
//        p.power_line_frequency = PAL::_50HZ;
//        p.vertical_flip = false;
//        p.filter_disparity = true;
//        p.filter_spots = true;
//        //p.fov_start = 180;
//        //p.fov_end = 270;
//        p.projection = PAL::PERSPECTIVE;
//        //p.mode = PAL::Mode::POINT_CLOUD_25D;
//        //p.mode = PAL::Mode::FAST_DEPTH;
//        PAL::SetCameraProperties(&p);
		timer.start(Period);
	}
}

void SpecificWorker::compute()
{
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    cv::Mat rgb, depth, right;

    // get image from camera
	PAL::GrabFrames(&left_img_write, &right_img_write, &depth_img_write);
	depth = cv::Mat(depth_img_write.rows, depth_img_write.cols, CV_32FC1, depth_img_write.Raw.f32_data);
	rgb = cv::Mat(left_img_write.rows, left_img_write.cols, CV_8UC3, left_img_write.Raw.u8_data);

    std::cout << "depth " << depth_img_write.cols << " " << depth_img_write.rows << " rgb " << left_img_write.cols << " " << left_img_write.rows << std::endl;
	//cv::resize(rgb, rgb , cv::Size(sc_width, sc_height));
	//cv::resize(depth, depth , cv::Size(rgb.cols, rgb.rows));
	cv::imshow( "PAL Occupancy Map", rgb);
	cv::waitKey(1);
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    mutex->lock();
		PAL::Image temp = left_img_write;
        left_img_write = left_img_read;
        left_img_read = left_img_write;
    mutex->unlock();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}


RoboCompCameraRGBDSimple::TRGBD SpecificWorker::CameraRGBDSimple_getAll(std::string camera)
{

}

RoboCompCameraRGBDSimple::TDepth SpecificWorker::CameraRGBDSimple_getDepth(std::string camera)
{
//implementCODE

}

RoboCompCameraRGBDSimple::TImage SpecificWorker::CameraRGBDSimple_getImage(std::string camera)
{
    RoboCompCameraRGBDSimple::TImage image;
    image.compressed = false; // opencv jpeg compression
    image.cameraID = 0;
    image.width = left_img_read.cols;
    image.height = left_img_read.rows;
    image.depth = 3;
    image.focalx = 0;
    image.focaly = 0;
    image.alivetime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    mutex->lock();
		auto ptr = reinterpret_cast<uint8_t*>(left_img_read.Raw.u8_data);
		image.image.insert(image.image.end(), ptr, ptr+sizeof(left_img_read.Raw.u8_data));
    mutex->unlock();
    return image;
}



/**************************************/
// From the RoboCompCameraRGBDSimplePub you can publish calling this methods:
// this->camerargbdsimplepub_pubproxy->pushRGBD(...)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

