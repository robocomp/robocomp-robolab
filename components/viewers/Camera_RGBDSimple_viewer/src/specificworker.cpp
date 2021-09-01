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

	this->Period = period;
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
       if (display_compressed) //Si quiero la imagen comprimida
       {
           std::string nada = "nada"; //test
           auto image = this->camerargbdsimple_proxy->getImage(nada);
           //std::cout << image.width << "x" << image.height << " size " << image.image.size()<<std::endl;
           cv::Mat frame(cv::Size(image.width, image.height), CV_8UC3, &image.image[0], cv::Mat::AUTO_STEP);
           cv::Mat frameCompr = cv::imdecode(image.image, -1);
           cv::imshow("RGB image", frameCompr);
           cv::waitKey(1);
       }
       else
       {
           std::string nada = "nada"; //test
           auto image = this->camerargbdsimple_proxy->getImage(nada);
           //std::cout << image.width << "x" << image.height << " size " << image.image.size()<<std::endl;
           cv::Mat frame(cv::Size(image.width, image.height), CV_8UC3, &image.image[0], cv::Mat::AUTO_STEP);
           cv::imshow("RGB image", frame);
           cv::waitKey(1);
       }
   }

   if (display_depth)
   {
       if (display_compressed) //Si quiero la imagen comprimida
       {
           std::string nada = "nada"; //test
           auto depth = this->camerargbdsimple_proxy->getDepth(nada);
           cv::Mat frame_depth = cv::imdecode(depth.depth, -1);

//           std::cout<< "Widht:" << frame_depth.rows << ", heigh: "<< frame_depth.cols<< ", size: " << frame_depth.elemSize() <<std::endl;

//           Voy a probar a mostrar los valores de todas las filas en la columna 300
//           for (int i = 0; i < 480; i++) {
//               std::cout << frame_depth.at<float>(i, 300) << std::endl;
//           }

           frame_depth.convertTo(frame_depth, CV_8UC3, 255. / 10, 0);
           applyColorMap(frame_depth, frame_depth, cv::COLORMAP_RAINBOW); //COLORMAP_HSV tb
           cv::imshow("Depth image ", frame_depth);
           cv::waitKey(1);
       }
       else
       {
           std::string nada = "nada"; //test
           auto depth = this->camerargbdsimple_proxy->getDepth(nada);
           cv::Mat frame_depth(cv::Size(depth.width, depth.height), CV_32FC1, &depth.depth[0], cv::Mat::AUTO_STEP);

//           std::cout<< "Widht:" << frame_depth.rows << ", heigh: "<< frame_depth.cols<< ", size: " << frame_depth.size <<std::endl;

//           Voy a probar a mostrar los valores de todas las filas en la columna 300
//           for (int i = 0; i < 480; i++) {
//               std::cout << frame_depth.at<float>(i, 300) << std::endl;
//           }

           frame_depth.convertTo(frame_depth, CV_8UC3, 255. / 10, 0);
           applyColorMap(frame_depth, frame_depth, cv::COLORMAP_RAINBOW); //COLORMAP_HSV tb
           //std::cout<<"Depth factor"<< depth.depthFactor<<std::endl;
           cv::imshow("Depth image ", frame_depth);
           cv::waitKey(1);
       }
   }

//   if (display_all) {
//           std::string nada = "nada"; //test
//
//           auto all = this->camerargbdsimple_proxy->getAll(nada);
//
//           cv::Mat frame_all(cv::Size(all.image.width, all.image.height), CV_8UC3, &all.image.image[0],
//                             cv::Mat::AUTO_STEP);
////           cv::Mat frameCompr_all = cv::imdecode(all.image.image, -1);
////           cv::imshow("RGB image", frameCompr_all);
//           cv::imshow("RGB image sin compr", frame_all);
//           cv::waitKey(1);
//
//           cv::Mat frame_depth_all(cv::Size(all.image.width, all.image.height), CV_32F, &all.image.image[0],
//                      cv::Mat::AUTO_STEP);
////           cv::Mat frame_depth_all = cv::imdecode(all.depth.depth, -1);
//           frame_depth_all.convertTo(frame_depth_all, CV_8UC3, 255. / 10, 0);
//           applyColorMap(frame_depth_all, frame_depth_all, cv::COLORMAP_RAINBOW); //COLORMAP_HSV tb
//           cv::imshow("Depth image sin compr", frame_depth_all);
//           cv::waitKey(1);
//
//   }


//    catch(const std::exception& e)
//    {
//        std::cerr << e.what() << '\n';
//    }

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

