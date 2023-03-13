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
		unsigned int flag = PAL::RESOLUTION | PAL::MODE | PAL::FD | PAL::NR | PAL::FILTER_SPOTS | PAL::VERTICAL_FLIP ;
		prop.mode = PAL::Mode::FAST_DEPTH; // The other available option is PAL::Mode::HIGH_QUALITY_DEPTH
		prop.resolution = {1120, 384};
		prop.mode = PAL::Mode::POINT_CLOUD_25D;
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
		timer.start(50);
	}
}

void SpecificWorker::compute()
{
	//std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	std::vector<PAL::Point> pc;
//	if (PAL::GetPointCloud(&pc) == PAL::SUCCESS)	// 384*128 points that come from a 640x224 depth map. x+,y+,z- upwards (centimeters)
//	{
//		std::cout << "points " << pc.size() << std::endl;
//		points_buffer.put(std::move(pc), [pc](auto &&input, auto &output)
//		{
//			output.points.reserve(pc.size());
//			std::transform(pc.begin(), pc.end(), std::back_inserter(output.points), [](auto &p)
//				{ return RoboCompCameraRGBDSimple::Point3D{p.x*10, p.y*10, -p.z*10};});
//			output.alivetime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//			output.period = 50;
//			output.compressed = false;
//		});
//	}

	// get image from camera
	PAL::Image left_img, right_img, depth_img;
	PAL::GrabFrames(&left_img, &right_img, &depth_img);
//
//	//std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
//	//std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
//
	image_buffer.put(std::move(left_img), [left_img](auto &&input, auto &output) // 640x224
   		{
			output.compressed = false; // opencv jpeg compression
			output.cameraID = 0;
			output.width = left_img.cols;
			output.height = left_img.rows;
			output.depth = 3;
			output.focalx = 0;
			output.focaly = 0;
			output.alivetime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
			auto ptr = reinterpret_cast<uint8_t*>(left_img.Raw.u8_data);
			output.image.insert(output.image.end(), ptr, ptr+left_img.cols*left_img.rows*3);
	    });
	depth_buffer.put(std::move(depth_img), [depth_img](auto &&input, auto &output)
	{
		output.compressed = false; // opencv jpeg compression
		output.cameraID = 0;
		output.width = depth_img.cols;
		output.height = depth_img.rows;
		output.focalx = 0;
		output.focaly = 0;
		output.alivetime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		output.depthFactor = 1;
		auto ptr = reinterpret_cast<std::uint8_t*>(depth_img.Raw.u8_data);
		output.depth.insert(output.depth.end(), ptr, ptr+depth_img.cols*depth_img.rows*sizeof(float));
	});

	fps.print("FPS: ");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
RoboCompCameraRGBDSimple::TRGBD SpecificWorker::CameraRGBDSimple_getAll(std::string camera)
{
	auto img = image_buffer.try_get();
	auto depth = depth_buffer.try_get();
	if(img.has_value() and depth.has_value())
	{
		RoboCompCameraRGBDSimple::TRGBD all;
		all.image.image.swap(img.value().image);
		all.depth.depth.swap(depth.value().depth);
		// faltan los puntos
		return all;
	}
	else
		return RoboCompCameraRGBDSimple::TRGBD();
}

RoboCompCameraRGBDSimple::TDepth SpecificWorker::CameraRGBDSimple_getDepth(std::string camera)
{
	if(auto depth = depth_buffer.try_get(); depth.has_value())
		return depth.value();
	else
		return RoboCompCameraRGBDSimple::TDepth();
}

RoboCompCameraRGBDSimple::TImage SpecificWorker::CameraRGBDSimple_getImage(std::string camera)
{
	if(auto img = image_buffer.try_get(); img.has_value())
		return img.value();
	else
		return RoboCompCameraRGBDSimple::TImage();
}

RoboCompCameraRGBDSimple::TPoints SpecificWorker::CameraRGBDSimple_getPoints(std::string camera)
{
	if(auto points = points_buffer.try_get(); points.has_value())
		return points.value();
	else
		return RoboCompCameraRGBDSimple::TPoints();
}


/**************************************/
// From the RoboCompCameraRGBDSimplePub you can publish calling this methods:
// this->camerargbdsimplepub_pubproxy->pushRGBD(...)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}
