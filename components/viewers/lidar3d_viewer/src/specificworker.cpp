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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		
		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
		
	}
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

void SpecificWorker::initialize()
{

	pc_red = this->configLoader.get<double>("pc_red");
	pc_green = this->configLoader.get<double>("pc_green");
	pc_blue = this->configLoader.get<double>("pc_blue");


	//chekpoint robocompUpdater
	std::cout << "Initialize worker" << std::endl;
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
		cv::createTrackbar("decimation filter", window_name, &slider_dec, 5, &SpecificWorker::on_decfilter, this);
		cv::setTrackbarMin("decimation filter", window_name, 1);
		// Checkboxes (0 o 1)
		cv::createTrackbar("Show Filtered (0/1)", window_name, &check_filtered, 1, nullptr);
		cv::createTrackbar("Show Invalid (0/1)", window_name, &check_invalid, 1, nullptr);


		// 3DViewer
        points = std::make_shared<std::vector<std::tuple<float, float, float>>>();
        colors = std::make_shared<std::vector<std::tuple<float, float, float>>>();
        
        viewer_3d = new Viewer(this, points, colors);
        viewer_3d->show();

		
	}


}

void SpecificWorker::compute()
{
    try
    {
        points->clear(); 
        colors->clear();

        // 1. Si está marcado "Invalid"
        if (check_invalid == 1)
        {
            auto ldata_invalid = lidar3d_proxy->getLidarData("invalid", qDegreesToRadians(slider_start-180), qDegreesToRadians(slider_len), slider_dec);
            for(const auto &p : ldata_invalid.points)
            {
                points->emplace_back(std::make_tuple(p.x / 1000, p.y / 1000, p.z / 1000));
                colors->emplace_back(std::make_tuple(1.0, 0.5, 0.5)); // Color rosado/rojo para inválidos
            }
        }

        // 2. Si está marcado "Filtered" (los puntos válidos estándar)
        if (check_filtered == 1)
        {
            // Usamos string vacío o "filtered" según tu componente Lidar
            auto ldata_filtered = lidar3d_proxy->getLidarData("", qDegreesToRadians(slider_start-180), qDegreesToRadians(slider_len), slider_dec);
            for(const auto &p : ldata_filtered.points)
            {
                points->emplace_back(std::make_tuple(p.x / 1000, p.y / 1000, p.z / 1000));
                colors->emplace_back(std::make_tuple(pc_red, pc_green, pc_blue)); // Color de configuración
            }
        }

        viewer_3d->update();
    }
    catch(const Ice::Exception &e)
    {
        std::cout << "Error reading from Lidar3D: " << e << std::endl;
    }
    cv::waitKey(1);
}

void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
	//computeCODE
	//
	//if (SUCCESSFUL)
    //  emmit goToRestore()
}

//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
	//computeCODE
	//Restore emergency component

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

