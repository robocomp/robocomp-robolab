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
    try
    {
        camera_name = params["camera_name"].value;
        this->Period = std::stoi(params.at("display_period").value);
        display_rgb = (params["display_rgb"].value == "true") or (params["display_rgb"].value == "True");
        display_depth = (params["display_depth"].value == "true") or (params["display_depth"].value == "True");
        display_compressed =
                (params["display_compressed"].value == "true") or (params["display_compressed"].value == "True");
        std::cout << "display_rgb " << display_rgb << " display_depth " << display_depth << " display_compressed "
                  << display_compressed << std::endl;
    }
    catch(const std::exception &e){ std::cout << e.what() << " Error reading params" << std::endl;};
    return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
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
   static int ant_period = 0;

   // Check if the image should be displayed or not.
   if (display_rgb)
   {
       try
       {
           // Retrieve the image from the camera
           auto image = this->camerargbdsimple_proxy->getImage("");
           if (image.width !=0 and image.height !=0)
           {
               // Check if the period of image has changed and update the timer if necessary
               if(ant_period != image.period and image.period > 0 and image.period < 1000)
               {
                   timer.setInterval(image.period);
                   ant_period = image.period;
               }

               // Check if the image is compressed
               if (image.compressed)
               {
                   cv::Mat frame(cv::Size(image.width, image.height), CV_8UC3, &image.image[0], cv::Mat::AUTO_STEP);
                   // Decode the compressed image
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
       { std::cout << e.what() << "Error connecting to CameraRGBDSimple::getImage"<< std::endl;}
   }

   if (display_depth)
   {
       try
       {
           // Retrieve the image from the camera
           auto depth = this->camerargbdsimple_proxy->getDepth("");
           if (depth.width != 0 and depth.height != 0)
           {
               // Check if the period of image has changed and update the timer if necessary
               if(ant_period != depth.period and depth.period > 0 and depth.period < 1000)
               {
                   timer.setInterval(depth.period);
                   ant_period = depth.period;
               }

               if (depth.compressed) //Si quiero la imagen comprimida
               {
                   // Se descomprime la imagen y se aplica un mapa de colores
                   cv::Mat frame_depth = cv::imdecode(depth.depth, -1);
                   frame_depth.convertTo(frame_depth, CV_8UC3, 255. / 10, 0);
                   applyColorMap(frame_depth, frame_depth, cv::COLORMAP_RAINBOW); //COLORMAP_HSV tb

                   // Se muestra la imagen en una ventana
                   cv::imshow("Depth image compress", frame_depth);
                   cv::waitKey(1);
               } else
               {
                   // Se crea una matriz de la imagen de profundidad y se aplica un mapa de colores
                   cv::Mat frame_depth(cv::Size(depth.width, depth.height), CV_32FC1, &depth.depth[0], cv::Mat::AUTO_STEP);
                   frame_depth.convertTo(frame_depth, CV_8UC3, 255. / 10, 0);
                   applyColorMap(frame_depth, frame_depth, cv::COLORMAP_RAINBOW); //COLORMAP_HSV tb

                   // Se muestra la imagen en una ventana
                   cv::imshow("Depth image ", frame_depth);
                   cv::waitKey(1);
               }
           }
           else
               qWarning() << "Warning. Empty depth frame";
       }
       catch(const std::exception &e)
       { std::cout << e.what() << "Error connecting to CameraRGBDSimple::getDepth"<< std::endl;}
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

