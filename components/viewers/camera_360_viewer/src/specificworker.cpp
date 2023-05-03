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
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
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
        try
        {
            img_params = this->camera360rgb_proxy->getImageParams();
        }
        catch(const Ice::Exception &ex) { qFatal("Error reading camera image");}

        // Create a window
        cv::namedWindow("ROI 360", 1);
        //Create trackbar to change width
        slider_width = 300;
        cv::createTrackbar("width", "ROI 360", &slider_width, img_params.width);

        //Create trackbar to change height
        slider_height = 300;
        cv::createTrackbar("height", "ROI 360", &slider_height, img_params.height);

        //Create trackbar to change x position
        slider_x = 300;
        cv::createTrackbar("x pos", "ROI 360", &slider_x, img_params.width);

        //Create trackbar to change y position
        slider_y = 300;
        cv::createTrackbar("y pos", "ROI 360", &slider_y, img_params.height);

        timer.start(Period);
	}
}

void SpecificWorker::compute()
{
    try
    {
        auto image = this->camera360rgb_proxy->getROI(slider_x, slider_y, slider_width, slider_height, 640, 640);
        if (not image.image.empty() and image.width !=0 and image.height !=0)
        {
            if (image.compressed)
            {
                cv::Mat frameCompr = cv::imdecode(image.image, -1);
                cv::imshow("ROI 360", frameCompr);
            }
            else
            {
                cv::Mat frame(cv::Size(image.width, image.height), CV_8UC3, &image.image[0]);
                cv::imshow("ROI 360", frame);
            }
            cv::waitKey(1);
        }
        else
            qWarning() << "No image received";
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    fps.print("FPS:");
}

//////////////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}




/**************************************/
// From the RoboCompCameraSimple you can call this methods:
// this->camerasimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraSimple you can use this types:
// RoboCompCameraSimple::TImage

