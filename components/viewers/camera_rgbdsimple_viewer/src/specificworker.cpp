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
    display_rgb = (params["display_rgb"].value == "true") or (params["display_rgb"].value == "True");
    display_depth = (params["display_depth"].value == "true") or (params["display_depth"].value == "True");
    display_compressed = (params["display_compressed"].value == "true") or (params["display_compressed"].value =="True");
    std::cout << "display_rgb " << display_rgb << " display_depth " << display_depth <<" display_compressed " << display_compressed << std::endl;
    return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
    this->Period = 33;
    std::cout <<"Period compute(ms): "<<this->Period<< " Frequency compute(Hz): " << (1.0/this->Period*1000)<<std::endl;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
	}
}

void SpecificWorker::compute()
{
   if (display_rgb)
   {
       try
       {
           auto image = this->camerargbdsimple_proxy->getImage("");
           if (image.width !=0 and image.height !=0)
           {
               if (image.compressed)
               {
                   cv::Mat frame(cv::Size(image.width, image.height), CV_8UC3, &image.image[0], cv::Mat::AUTO_STEP);
                   cv::Mat frameCompr = cv::imdecode(image.image, -1);
                   cv::imshow("RGB image compress", frameCompr);
                   cv::waitKey(1);
               } else
               {
                   cv::Mat frame(cv::Size(image.width, image.height), CV_8UC3, &image.image[0], cv::Mat::AUTO_STEP);
                   cv::imshow("RGB image", frame);
                   cv::waitKey(1);
               }
           }
           else
               qInfo() << "No image received";
       }
       catch(const std::exception &e)
       { std::cout << e.what() << "Error connecting to CameraRGBDSimple"<< std::endl;}
   }

   if (display_depth)
   {
       try
       {
           auto depth = this->camerargbdsimple_proxy->getDepth("");
           if (depth.width != 0 and depth.height != 0)
           {
               if (depth.compressed) //Si quiero la imagen comprimida
               {
                   cv::Mat frame_depth = cv::imdecode(depth.depth, -1);
                   frame_depth.convertTo(frame_depth, CV_8UC3, 255. / 10, 0);
                   applyColorMap(frame_depth, frame_depth, cv::COLORMAP_RAINBOW); //COLORMAP_HSV tb
                   cv::imshow("Depth image compress", frame_depth);
                   cv::waitKey(1);
               } else
               {
                   cv::Mat frame_depth(cv::Size(depth.width, depth.height), CV_32FC1, &depth.depth[0], cv::Mat::AUTO_STEP);
                   frame_depth.convertTo(frame_depth, CV_8UC3, 255. / 10, 0);
                   applyColorMap(frame_depth, frame_depth, cv::COLORMAP_RAINBOW); //COLORMAP_HSV tb
                   cv::imshow("Depth image ", frame_depth);
                   cv::waitKey(1);
               }
           }
           else
               qWarning() << "Warning. Empty depth frame";
       }
       catch(const std::exception &e)
       { std::cout << e.what() << "Error connecting to CameraRGBDSimple"<< std::endl;}
   }

    fps.print("FPS: ");
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

