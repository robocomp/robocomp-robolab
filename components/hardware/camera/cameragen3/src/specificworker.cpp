/*
 *    Copyright (C)2020 by YOUR NAME HERE
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
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	emit t_compute_to_finalize();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	defaultMachine.start();
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	vision.initialize();
	vision.start();
	
	this->Period = period;
	timer.start(33);
	emit this->t_initialize_to_compute();

}

void SpecificWorker::compute()
{
	
	auto [res, img] = vision.publish();
	if(res)
	{
		//cv::Mat image(480, 640, CV_8UC3, &img.image[0]);
		//cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
		//cv::Mat imgdst(608, 608, CV_8UC3);
		//cv::resize(image, imgdst, imgdst.size(), 0, 0, cv::INTER_LINEAR);
		//cv::imshow("CameraGen3", imgdst);

		try
		{
			//RoboCompYoloServer::TImage yimg{ imgdst.cols, imgdst.rows, 3};
			RoboCompYoloServer::TImage yimg{img.width, img.height, img.depth};
			yimg.image.resize(yimg.width*yimg.height*yimg.depth);
			std::swap(yimg.image, img.image);
			//yimg.image.resize(yimg.width*yimg.height*yimg.depth);
			//memcpy(&yimg.image[0], imgdst.data, yimg.width*yimg.height*yimg.depth);
			auto objects = yoloserver_proxy->processImage(yimg);
			RoboCompCameraRGBDSimple::TDepth rgbdDepth;
			camerargbdsimpleyolopub_pubproxy->pushRGBDYolo(img, rgbdDepth, objects);
			qDebug() << "Objects " << objects.size();
		}
		catch(const std::exception& e)
		{
			std::cerr << e.what() << '\n';
		}
	}
}

void SpecificWorker::sm_compute()
{
	//std::cout<<"Entered state compute"<<std::endl;
	compute();
}

void SpecificWorker::sm_initialize()
{
	std::cout<<"Entered initial state initialize"<<std::endl;
}

void SpecificWorker::sm_finalize()
{
	std::cout<<"Entered final state finalize"<<std::endl;
}

