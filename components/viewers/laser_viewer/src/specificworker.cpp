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
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

    robot_image = cv::imread("giraff.png", -1);
    cv::resize(robot_image, robot_image, cv::Size(50,50));
    //cv::flip(robot_image, robot_image, -1);

	this->Period = period;
	if(this->startup_check_flag)
		this->startup_check();
	else
		timer.start(Period);
}

void SpecificWorker::compute()
{
    try
    {
        new_laser= laser_proxy->getLaserData();
        draw_laser(new_laser);
    }
    catch(const Ice::Exception &e)
    { std::cout << e.what() << std::endl; }
    fps.print("FPS: ");
}

void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata)
{
    // Check if laser data is empty
    if(ldata.empty()) { qWarning() << "Laser empty"; return;}


    // Create an 8-bit 3-channel image with the specified size
    cv::Mat laser_img(cv::Size(lado, lado), CV_8UC3);
    // Set the color for the laser image
    auto laser_color = cv::Scalar(198, 220, 220);
    // Set the background color for the laser image
    laser_img = cv::Scalar(255, 255, 255);
    // Create a vector to store the points for the laser image
    std::vector<cv::Point> fillContSingle;
    
    // Add the center point to the vector
    fillContSingle.push_back(cv::Point{semilado,semilado});
    // Iterate through the laser data
    for(auto &&l : ldata)
    {
        // Check if dist and angle has valid values
        if(!(isfinite(l.dist) && isfinite(l.angle))) continue;

        // Calculate the x and y coordinates for the point
        int x = l.dist * sin(l.angle) * scale + semilado;
        int y = semilado - l.dist * cos(l.angle) * scale;

        // Add the point to the vector
        fillContSingle.push_back(cv::Point(x,y));
    }

    // Add the center point to the vector
    fillContSingle.push_back(cv::Point(semilado,semilado));
    // Create a vector of vectors to store the points
    std::vector<std::vector<cv::Point> > fillContAll;
     // Add the points vector to the vector of vectors
    fillContAll.push_back(fillContSingle);
    // Fill the laser image with the points and color
    cv::fillPoly( laser_img, fillContAll, laser_color);

    // Copy the robot image to the laser image
    robot_image.copyTo(laser_img(cv::Rect(semilado - robot_image.cols/2, semilado - robot_image.rows/2, robot_image.cols, robot_image.rows)));

    // Draw circles on the laser image
    for(auto &&i : iter::range(10))
    {
        cv::circle(laser_img, cv::Point{semilado, semilado}, i*scale*radius, cv::Scalar(10, 200, 10));
    }
    // Draw spikes on the laser image
    for(auto landa : iter::range(0.0, M_PI*2.0, M_PI*2/8))
    {
        auto p = cv::Point(semilado + semilado * cos(landa), semilado + semilado * sin(landa));
        cv::line(laser_img, cv::Point{semilado, semilado}, p, cv::Scalar(10, 200, 10));
    }

    // Show the laser image in a window
    cv::imshow("Laser", laser_img);
    cv::waitKey(2);
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}


RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState)
{
    return RoboCompLaser::TLaserData();
}

RoboCompLaser::LaserConfData SpecificWorker::Laser_getLaserConfData()
{
    return RoboCompLaser::LaserConfData();
}

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserData()
{
    std::lock_guard<std::mutex> lg(my_mutex);
    return new_laser;
}

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

