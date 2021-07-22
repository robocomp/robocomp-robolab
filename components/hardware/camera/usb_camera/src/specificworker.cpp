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

	if( auto success = capture.open("/dev/video2"); success != true)
    {
	    qWarning() << __FUNCTION__ << " No camera found";
	    std::terminate();
    }
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(50);

	this->Period = 50;
	if(this->startup_check_flag)
		this->startup_check();
	else
		timer.start(Period);
}

void SpecificWorker::compute()
{
    my_mutex.lock();
        capture >> frame;
        //qInfo() << frame.total() * frame.elemSize();
        cv::imencode(".jpg", frame, buffer, compression_params);
        //qInfo() << "raw: " << frame.total() * frame.elemSize() << "compressed: " << buffer.size() << " Ratio:" << frame.total() * frame.elemSize()/buffer.size();
    my_mutex.unlock();
    //cv::imshow("USB Camera", frame);
    //cv::Mat dest=cv::imdecode(buffer, -1);
   // cv::imshow("USB Camera comprimido", dest);
    //cv::waitKey(2); // waits to display frame

    //fps.print("FPS:");
}
/////////////////////////////////////////////////////////////////////

RoboCompCameraSimple::TImage SpecificWorker::CameraSimple_getImage()
{
    std::lock_guard<std::mutex> lg(my_mutex);
    RoboCompCameraSimple::TImage res;
    res.depth = 3;
    res.height = frame.rows;
    res.width = frame.cols;
    res.compressed=true;
    //res.image.assign(frame.data, frame.data + (frame.rows*frame.cols*3));
    res.image.assign(buffer.begin(), buffer.end());
    //copy(buffer.begin(),buffer.end(),res.image.begin());
    return res;
}

///////////////////////////////////////////////////////////////////77
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}




