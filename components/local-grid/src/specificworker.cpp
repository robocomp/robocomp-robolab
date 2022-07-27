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
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    try
    {
        auto left_x = std::stod(params.at("left_x").value);
        auto top_y = std::stod(params.at("top_y").value);
        auto width = std::stod(params.at("width").value);
        auto height = std::stod(params.at("height").value);
        auto tile = std::stod(params.at("tile").value);
        qInfo() << __FUNCTION__ << " Read parameters: " << left_x << top_y << width << height << tile;
        viewer_dimensions = QRectF(left_x, top_y, width, height);
        consts.tile_size = tile;
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
        viewer = new AbstractGraphicViewer(this->beta_frame, this->viewer_dimensions);
        this->resize(900,450);
        const auto &[rp, re] = viewer->add_robot(consts.robot_length, consts.robot_length);
        robot_polygon = rp;
        laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
        laser_in_robot_polygon->setPos(0, 190);     // move this to abstract

        local_grid.initialize(Local_Grid::Ranges{-M_PI, M_PI, 0.1}, Local_Grid::Ranges{0.f, 4000.f, 5}, &viewer->scene);

        timer.start(Period);
	}

}

void SpecificWorker::compute()
{
    qInfo() << __FUNCTION__ << "compute";
//	try
//	{
//	  auto ldata = laser_proxy->getLaserData();
//	}
//	catch(const Ice::Exception &e){std::cout << "Error reading from Camera" << e << std::endl;}
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

/**************************************/
// From the RoboCompCameraRGBDSimple you can call this methods:
// this->camerargbdsimple_proxy->getAll(...)
// this->camerargbdsimple_proxy->getDepth(...)
// this->camerargbdsimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

