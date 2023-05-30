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
int SpecificWorker::slider_start;
int SpecificWorker::slider_len;
//int SpecificWorker::slider_len;
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
	// Uncomment if there's too many debug messages
	// but it removes the possibility to see the messages
	// shown in the console with qDebug()
//	QLoggingCategory::setFilterRules("*.debug=false\n");
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
		// 3DViewer
        points = std::make_shared<std::vector<std::tuple<float, float, float>>>();
        colors = std::make_shared<std::vector<std::tuple<float, float, float>>>();
        
        viewer_3d = new Viewer(this, points, colors);
        viewer_3d->show();

		timer.start(50);
	}
    cv::namedWindow("LIDAR VARIABLE", cv::WINDOW_AUTOSIZE);
    slider_start = 300;
    slider_len = 100;
//    auto src1 = cv::imread( "LinuxLogo.png");
    cv::createTrackbar("start", "LIDAR VARIABLE", &slider_start, 900 , &SpecificWorker::on_start, this);
    cv::createTrackbar("len", "LIDAR VARIABLE", &slider_len, 900 , &SpecificWorker::on_len, this);
}

void SpecificWorker::compute()
{
	try
	{
		points->clear(); colors->clear();
		auto ldata = lidar3d_proxy->getLidarData(slider_start, slider_len);
		qInfo() << ldata.size();
		points->resize(ldata.size());
		colors->resize(points->size());
		
	    for(const auto &[i, p]: ldata | iter::enumerate)
		{
			points->operator[](i) = std::make_tuple(p.x, p.y, p.z);
			colors->operator[](i) = std::make_tuple(pc_red, pc_green, pc_blue);
		}

		viewer_3d->updateGL();

	}

	catch(const Ice::Exception &e)
	{
	  std::cout << "Error reading from Lidar3D" << e << std::endl;
	}
//    cv::imshow("LIDAR VARIABLE", src1);
    cout << "VALOR SLIDER START: " << slider_start << std::endl;
    cv::waitKey(1);

}

void SpecificWorker::on_start(int pos, void *data)
{
    slider_start = slider_start;
}
void SpecificWorker::on_len(int pos, void *data)
{
    slider_len = slider_len;
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

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint

