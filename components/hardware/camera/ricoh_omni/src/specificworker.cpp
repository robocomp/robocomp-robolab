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
    try
    {
        //Parametros por el config
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
    std::cout << "Initializing worker" << std::endl;

    this->Period = 33;
	if(this->startup_check_flag)
		this->startup_check();
	else
    {
        if( auto success = capture.open("thetauvcsrc mode=4K ! queue ! h264parse ! nvdec ! gldownload ! queue ! videoconvert n-threads=2 ! video/x-raw,format=BGR ! queue ! appsink drop=true sync=false", cv::CAP_GSTREAMER);
                success != true)
        {
            qWarning() << __FUNCTION__ << " Error connecting. No camera found";
            std::terminate();
        }
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params.push_back(50);
        capture >> cv_frame;
        MAX_WIDTH = cv_frame.cols;
        MAX_HEIGHT = cv_frame.rows;
        DEPTH = cv_frame.elemSize();
        timer.start(Period);
    }
}

void SpecificWorker::compute()
{
    capture >> cv_frame;
    if(pars.display)
    {
        cv::imshow("USB Camera", cv_frame);
        cv::waitKey(1); // waits to display frame
    }
    buffer_image.put(std::move(cv_frame));
    fps.print("FPS:");
}
/////////////////////////////////////////////////////////////////////
RoboCompCameraRGBDSimple::TImage SpecificWorker::Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)
{
    if(sx == -1)
        sx = MAX_WIDTH;
    if(sy == -1)
        sy = MAX_HEIGHT;
    if(cx == -1)
        cx = (int)(MAX_WIDTH/2);
    if(cy == -1)
        cy = int(MAX_HEIGHT/2);
    if(roiwidth == -1)
        roiwidth = MAX_WIDTH;
    if(roiheight == -1)
        roiheight = MAX_HEIGHT;

    auto img = buffer_image.get();
    cv::Mat dst, rdst;
    img(cv::Rect(cx - (int) (sx / 2), cy - (int) (sy / 2), sx, sy)).copyTo(dst);
    cv::resize(dst, rdst, cv::Size(roiwidth, roiheight), cv::INTER_LINEAR);
    //qInfo() << "requested " << cx - (int) (sx / 2) << cy - (int) (sy / 2) << "resized " << rdst.rows << rdst.cols;
    RoboCompCameraRGBDSimple::TImage res;
    if (pars.compressed)
    {
        std::vector<uchar> buffer;
        cv::imencode(".jpg", rdst, buffer, compression_params);
        res.image = buffer;
        res.compressed = true;
    } else
    {
        res.image.assign(rdst.data, rdst.data + (rdst.total() * rdst.elemSize()));
        res.compressed = false;
    }
    res.depth = rdst.channels();
    res.height = rdst.rows;
    res.width = rdst.cols;
    return res;
}

///////////////////////////////////////////////////////////////////77
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}


/**************************************/
// From the RoboCompCameraSimple you can use this types:
// RoboCompCameraSimple::TImage
