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
        //Parametros por el config
        pars.device  = params.at("device").value;
        pars.display = params.at("display").value == "true" or (params.at("display").value == "True");
        pars.compressed = params.at("compressed").value == "true" or (params.at("compressed").value == "True");
        std::cout << "Params: device" << pars.device << " display " << pars.display << " compressed: " << pars.compressed << std::endl;
    }
    catch(const std::exception &e)
    { std::cout << e.what() << " Error reading config params" << std::endl;};
    return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	if( auto success = capture.open(pars.device); success != true)
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
        if(pars.compressed)
            cv::imencode(".jpg", frame, buffer, compression_params);
    my_mutex.unlock();

    if(pars.display)
    {
        cv::imshow("USB Camera", frame);
        cv::waitKey(2); // waits to display frame
    }

    fps.print("Compression: " + std::to_string(frame.total() * frame.elemSize()/buffer.size()));
    //fps.print("");

}
/////////////////////////////////////////////////////////////////////

RoboCompCameraSimple::TImage SpecificWorker::CameraSimple_getImage()
{
    qInfo() << __FUNCTION__ << "hola";
    std::lock_guard<std::mutex> lg(my_mutex);
    RoboCompCameraSimple::TImage res;
    res.depth = frame.channels();
    res.height = frame.rows;
    res.width = frame.cols;
    res.compressed = pars.compressed;
    if(res.compressed)
        res.image = buffer;
    else
        res.image.assign(frame.data, frame.data + (frame.total() * frame.elemSize()));
    return res;
}

///////////////////////////////////////////////////////////////////77
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}




