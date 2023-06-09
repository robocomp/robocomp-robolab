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
        pars.simulator = params.at("simulator").value == "true" or (params.at("simulator").value == "True");
        pars.compressed = params.at("compressed").value == "true" or (params.at("compressed").value == "True");
        std::cout << "Params: device" << pars.device << " display " << pars.display << " compressed: " << pars.compressed << " simulator: " << pars.simulator << std::endl;
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
        bool activated_camera = false;
        while(!activated_camera)
        {
            if(!pars.simulator)
            {
                if( auto success = capture.open("thetauvcsrc mode=2K ! queue ! h264parse ! nvdec ! gldownload ! queue ! videoconvert n-threads=2 ! video/x-raw,format=BGR ! queue ! appsink drop=true sync=false", cv::CAP_GSTREAMER);
                        success != true)
                {
                    qWarning() << __FUNCTION__ << " Error connecting. No camera found";
                }
                else
                {
                    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
                    compression_params.push_back(50);
                    capture >> cv_frame;
                    MAX_WIDTH = cv_frame.cols;
                    MAX_HEIGHT = cv_frame.rows;
                    activated_camera = true;
                }
            }
            else
            {
                try
                {
                    qInfo() << "Trying to connect to Camera360RGB in simulator";
                    image = this->camera360rgb_proxy->getROI(-1, -1, -1, -1, -1, -1);
                    cv_frame = cv::Mat (cv::Size(image.width, image.height), CV_8UC3, &image.image[0]);
                    MAX_WIDTH = image.width;
                    MAX_HEIGHT = image.height;
                    activated_camera = true;
                }
                catch(const std::exception &e){ std::cout << e.what() << std::endl;};
            }
        }

        DEPTH = cv_frame.elemSize();
        timer.start(Period);
    }
}

void SpecificWorker::compute()
{
    if(not pars.simulator)
        capture >> cv_frame;
    else
    {
        try
        {
            image = this->camera360rgb_proxy->getROI(-1, -1, -1, -1, -1, -1);
            cv_frame = cv::Mat(cv::Size(image.width, image.height), CV_8UC3, &image.image[0]);
        }
        catch (const std::exception &e)
        { std::cout << e.what() << " Error reading 360 camera " << std::endl; }
    }

    if(pars.display)
    {
        cv::imshow("USB Camera", cv_frame);
        cv::waitKey(1); // waits to display frame
    }
    buffer_image.put(std::move(cv_frame)); // TODO: Replace by tuple with timestamp
    fps.print("FPS:");
}
/////////////////////////////////////////////////////////////////////
RoboCompCamera360RGB::TImage SpecificWorker::Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)
{
    RoboCompCamera360RGB::TImage res;
//    std::cout << "REQUIRED IMAGE: " << sx << " " << sy << " " << cx << " " << cy << " " << roiwidth << " " << roiheight << std::endl;
    if(sx == 0 || sy == 0)
    {
        std::cout << "No size. Sending complete image" << std::endl;
        sx = MAX_WIDTH; sy = MAX_HEIGHT;
        cx = (int)(MAX_WIDTH/2); cy = int(MAX_HEIGHT/2);
    }
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

    // Get image from doublebuffer
    auto img = buffer_image.get();

    // Check if y is out of range. Get max or min values in that case
    if((cy - (int) (sy / 2)) < 0)
    {
//        std::cout << "CASO 1" << std::endl;
        sx = (int) ((float) sx / (float) sy * 2 * cy );
        sy = 2*cy;
    }
    else if((cy + (int) (sy / 2)) >= MAX_HEIGHT)
    {
//        std::cout << "CASO 2" << std::endl;
        sx = (int) ((float) sx / (float) sy * 2 * (MAX_HEIGHT - cy) );
        sy = 2 * (MAX_HEIGHT - cy);
    }

//    std::cout << "Converted IMAGE: " << sx << " " << sy << " " << cx << " " << cy << " " << roiwidth << " " << roiheight << std::endl;

    // Check if x is out of range. Add proportional image section in that case
    cv::Mat x_out_image_left, x_out_image_right, dst, rdst;;
    if((cx - (int) (sx / 2)) < 0)
    {
//        std::cout << "CENTER MINOR THAN 0" << std::endl;
//        std::cout << "LEFT SECTION " << MAX_WIDTH - 1 - abs (cx - (int) (sx / 2)) << std::endl;
//        std::cout << "RIGHT SECTION " << cx + (int) (sx / 2) <<std::endl;
        img(cv::Rect(MAX_WIDTH - 1 - abs (cx - (int) (sx / 2)), cy - (int) (sy / 2), abs (cx - (int) (sx / 2)), sy)).copyTo(x_out_image_left);
        img(cv::Rect(0, cy - (int) (sy / 2),  cx + (int) (sx / 2), sy)).copyTo(x_out_image_right);
        cv::hconcat(x_out_image_left, x_out_image_right, dst);
    }

    else if((cx + (int) (sx / 2)) > MAX_WIDTH)
    {
//        std::cout << "CENTER MAYOR THAN MAX_WIDTH" << std::endl;
        img(cv::Rect(cx - (int) (sx / 2) - 1, cy - (int) (sy / 2), MAX_WIDTH - cx + (int) (sx / 2), sy)).copyTo(x_out_image_left);
        img(cv::Rect(0, cy - (int) (sy / 2), cx + (int) (sx / 2) - MAX_WIDTH, sy)).copyTo(x_out_image_right);
        cv::hconcat(x_out_image_left, x_out_image_right, dst);
    }

    else{ img(cv::Rect(cx - (int) (sx / 2), cy - (int) (sy / 2), sx, sy)).copyTo(dst); }

    cv::resize(dst, rdst, cv::Size(roiwidth, roiheight), cv::INTER_LINEAR);
    //qInfo() << "requested " << cx - (int) (sx / 2) << cy - (int) (sy / 2) << "resized " << rdst.rows << rdst.cols;

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
    res.period = fps.get_period();
    res.alivetime = duration_cast< milliseconds >(system_clock::now().time_since_epoch()).count();
    res.depth = rdst.channels();
    res.height = rdst.rows;
    res.width = rdst.cols;
    res.roi = RoboCompCamera360RGB::TRoi{.xcenter=cx, .ycenter=cy, .xsize=sx, .ysize=sy, .finalxsize=res.width, .finalysize=res.height};
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

