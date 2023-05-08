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

int SpecificWorker::slider_width;
int SpecificWorker::slider_height;
int SpecificWorker::slider_x;
int SpecificWorker::slider_y;
SpecificWorker::Fovea SpecificWorker::fovea;

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
            initial_img = this->camera360rgb_proxy->getROI(-1, -1, -1, -1, -1, -1);
            // Create a window
            cv::namedWindow("ROI 360", 1);
            fovea.max_height = initial_img.height;
            fovea.max_width = initial_img.width;
            fovea.r2full_x = (float)MAIN_IMAGE_WIDTH/fovea.max_width;
            fovea.r2full_y = (float)MAIN_IMAGE_HEIGHT/fovea.max_height;

            //Create trackbar to change width
            slider_width = 300;
            cv::createTrackbar("width", "ROI 360", &slider_width, initial_img.width,&SpecificWorker::on_width, this);

            //Create trackbar to change height
            slider_height = 300;
            cv::createTrackbar("height", "ROI 360", &slider_height, initial_img.height, &SpecificWorker::on_height, this );

            //Create trackbar to change x position
            slider_x = initial_img.width/2;
            cv::createTrackbar("x pos", "ROI 360", &slider_x, initial_img.width, &SpecificWorker::on_cx, this);

            //Create trackbar to change y position
            slider_y = initial_img.height/2;
            cv::createTrackbar("y pos", "ROI 360", &slider_y, initial_img.height, &SpecificWorker::on_cy, this);
        }
        catch(const Ice::Exception &ex) { qFatal("Error reading camera image in Initialize");}

        timer.start(Period);
	}
}

void SpecificWorker::compute()
{
    try
    {
        auto full = this->camera360rgb_proxy->getROI(-1, -1, -1, -1, MAIN_IMAGE_WIDTH, MAIN_IMAGE_HEIGHT);
        cv::Mat full_mat(cv::Size(full.width, full.height), CV_8UC3, &full.image[0]);
        auto roi = this->camera360rgb_proxy->getROI(slider_x, slider_y, slider_width, slider_height, 320, 320);
        cv::Mat roi_mat(cv::Size(roi.width, roi.height), CV_8UC3, &roi.image[0]);
        cv::rectangle(full_mat, cv::Rect((int)(fovea.r2full_x*(slider_x-slider_width/2)),
                                                  (int)(fovea.r2full_y*(slider_y-slider_height/2)),
                                                     (int)(fovea.r2full_x*slider_width),
                                                     (int)(fovea.r2full_y*slider_height)),
                                                     cv::Scalar(255, 0, 0),
                                            3);

        cv::imshow("Full View", full_mat);
        cv::imshow("ROI 360", roi_mat);
        cv::waitKey(1);
    }
    catch(const Ice::Exception& e)
    {  std::cout << e.what() << std::endl;   }

//    try
//    {
//        auto image = this->camera360rgb_proxy->getROI(slider_x, slider_y, slider_width, slider_height, 640, 640);
//        if (not image.image.empty() and image.width !=0 and image.height !=0)
//        {
//            if (image.compressed)
//            {
//                cv::Mat frameCompr = cv::imdecode(image.image, -1);
//                cv::imshow("ROI 360", frameCompr);
//            }
//            else
//            {
//                cv::Mat frame(cv::Size(image.width, image.height), CV_8UC3, &image.image[0]);
//                cv::imshow("ROI 360", frame);
//            }
//            cv::waitKey(1);
//        }
//        else
//            qWarning() << "No image received";
//    }
//    catch(const std::exception& e)
//    {
//        std::cerr << e.what() << '\n';
//    }
    fps.print("FPS:");
}


void SpecificWorker::on_cx(int pos, void *data)
{
    if(slider_x + slider_width/2 > fovea.max_width or slider_x - slider_width/2 < 0)
        slider_x = fovea.cx;
    else
        fovea.cx = slider_x;
}
void SpecificWorker::on_cy(int pos, void *data)
{
    if(slider_y + slider_height/2 > fovea.max_height or slider_y - slider_height/2 < 0)
        slider_y = fovea.cy;
    else
        fovea.cy = slider_y;
}
void SpecificWorker::on_height(int pos, void *data)
{
    if(slider_y + slider_height/2 > fovea.max_height or slider_y - slider_height/2 < 0)
        slider_height = fovea.height;
    else
        fovea.height = slider_height;
}
void SpecificWorker::on_width(int pos, void *data)
{
    if(slider_x + slider_width/2 > fovea.max_width or slider_x - slider_width/2 < 0)
        slider_width = fovea.width;
    else
        fovea.width = slider_width;
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

