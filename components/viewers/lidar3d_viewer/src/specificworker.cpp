/*
 *    Copyright (C) 2023 by YOUR NAME HERE
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
#include <cppitertools/enumerate.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

int SpecificWorker::slider_start = 180;
int SpecificWorker::slider_len = 100;
int SpecificWorker::slider_z = 2000;
int SpecificWorker::slider_dec = 1;

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
	try
    {
        pc_red = std::stod(params.at("pc_red").value);
        pc_green = std::stod(params.at("pc_green").value);
        pc_blue = std::stod(params.at("pc_blue").value);
    }
    catch(const std::exception &e){ std::cout << e.what() << std::endl;}

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


		window_name = "3D LIDAR Viewer";
		cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

		cv::createTrackbar("start", window_name, &slider_start, 360 , &SpecificWorker::on_start, this);
		cv::createTrackbar("len", window_name, &slider_len, 360 , &SpecificWorker::on_len, this);
		cv::createTrackbar("z filter", window_name, &slider_z, 4000, &SpecificWorker::on_zfilter, this);
		cv::createTrackbar("decimation filter", window_name, &slider_dec, 5, &SpecificWorker::on_decfilter, this);
		cv::setTrackbarMin("decimation filter", window_name, 1);


		// 3DViewer
        points = std::make_shared<std::vector<std::tuple<float, float, float>>>();
        colors = std::make_shared<std::vector<std::tuple<float, float, float>>>();
        
        viewer_3d = new Viewer(this, points, colors);
        viewer_3d->show();

		timer.start(50);
		
	}


}

void SpecificWorker::compute()
{
	try
	{
		cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
		points->clear(); colors->clear();
		auto ldata = lidar3d_proxy->getLidarData("", qDegreesToRadians(slider_start-180), qDegreesToRadians(slider_len), slider_dec);
		points->resize(ldata.points.size());
		colors->resize(points->size());
		
		for(const auto &[i, p]: ldata.points | iter::enumerate)
		{
            if(p.z > (slider_z - 2000))
            {
                points->operator[](i) = std::make_tuple(p.x / 1000, p.y / 1000, p.z / 1000);
                colors->operator[](i) = std::make_tuple(pc_red, pc_green, pc_blue);
            }
		}
		viewer_3d->update();
	}

	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from Lidar3D" << e << std::endl;
	}
    cv::waitKey(1);

}

void SpecificWorker::on_start(int pos, void *data)
{
    auto *worker = static_cast<SpecificWorker*>(data);
    worker->slider_start = pos;
}

void SpecificWorker::on_len(int pos, void *data)
{
    auto *worker = static_cast<SpecificWorker*>(data);
    worker->slider_len = pos;
}

void SpecificWorker::on_zfilter(int pos, void *data)
{
    auto *worker = static_cast<SpecificWorker*>(data);
    worker->slider_z = pos;
}

void SpecificWorker::on_decfilter(int pos, void *data)
{
    auto *worker = static_cast<SpecificWorker*>(data);
    worker->slider_dec = pos;
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}






/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)
// this->lidar3d_proxy->getLidarDataWithThreshold2d(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData

