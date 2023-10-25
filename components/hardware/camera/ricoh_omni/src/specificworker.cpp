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
        pars.orin = params.at("orin").value == "true" or (params.at("orin").value == "True");
        pars.compressed = params.at("compressed").value == "true" or (params.at("compressed").value == "True");
        pars.time_offset =std::stof(params.at("time_offset").value);
        std::cout << "Params: device" << pars.device << " display " << pars.display << " compressed: " << pars.compressed << " simulator: " << pars.simulator << "offset "<< pars.time_offset <<std::endl;
    }
    catch(const std::exception &e)
    { std::cout << e.what() << " Error reading config params" << std::endl;};
    return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initializing worker" << std::endl;
    
    if (pars.orin)
    	pipeline = "thetauvcsrc mode=2K ! queue ! h264parse ! nvv4l2decoder ! nvvidconv ! queue ! videoconvert n-threads=2 ! video/x-raw,format=BGR ! queue ! appsink drop=true sync=false";
    else
    	pipeline = "thetauvcsrc mode=2K ! queue ! h264parse ! nvdec ! gldownload ! queue ! videoconvert n-threads=2 ! video/x-raw,format=BGR ! queue ! appsink drop=true sync=false";
    //if (pars.orin)
    	//pipeline = "thetauvcsrc mode=2K ! h264parse ! nvv4l2decoder ! nvvidconv ! videoconvert n-threads=2 ! video/x-raw,format=BGR ! appsink drop=true sync=false";
    //else
    	//pipeline = "thetauvcsrc mode=2K ! h264parse ! nvdec ! gldownload ! videoconvert n-threads=2 ! video/x-raw,format=BGR ! appsink drop=true sync=false";
    this->Period = 50;
	if(this->startup_check_flag)
		this->startup_check();
	else
    {
        
        while(!activated_camera)
        {
            if(!pars.simulator)
            {
                if( auto success = capture.open(pipeline, cv::CAP_GSTREAMER);
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
                catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;};
            }
        }
        capture_time = -1;
        DEPTH = cv_frame.elemSize();
        timer.start(Period);
    }
}

void SpecificWorker::compute()
{
    if(not pars.simulator)
    {
        capture >> cv_frame;
        capture_time = duration_cast< milliseconds >(system_clock::now().time_since_epoch()).count() - pars.time_offset;
    }
    else        // Simulator
    {
        try
        {
            image = this->camera360rgb_proxy->getROI(-1, -1, -1, -1, -1, -1);
            cv_frame = cv::Mat(cv::Size(image.width, image.height), CV_8UC3, &image.image[0]);
            capture_time = image.alivetime;
            // self adjusting period to remote image source
            self_adjust_period(image.period);
        }
        catch (const Ice::Exception &e)
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

    void SpecificWorker::self_adjust_period(int new_period)
    {
        if(abs(new_period - this->Period) < 2)
            return;
        if(new_period > this->Period)
        {
            this->Period += 1;
            timer.setInterval(this->Period);
        } else
        {
            this->Period -= 1;
            this->timer.setInterval(this->Period);
        }
    }
/////////////////////////////////////////////////////////////////////
////////////// Interface
/////////////////////////////////////////////////////////////////////

RoboCompCamera360RGB::TImage SpecificWorker::Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)
{
    RoboCompCamera360RGB::TImage res;
    if (this->activated_camera)
    {
        //std::cout << "REQUIRED IMAGE: " << sx << " " << sy << " " << cx << " " << cy << " " << roiwidth << " " << roiheight << std::endl;
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
        auto img = buffer_image.get_idemp();      // TODO: change to try_get() or get_idemp()

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

        // Check if x is out of range. Add proportional image section in that case
        cv::Mat x_out_image_left, x_out_image_right, dst, rdst;;
        if((cx - (int) (sx / 2)) < 0)
        {
            img(cv::Rect(MAX_WIDTH - 1 - abs (cx - (int) (sx / 2)), cy - (int) (sy / 2), abs (cx - (int) (sx / 2)), sy)).copyTo(x_out_image_left);
            img(cv::Rect(0, cy - (int) (sy / 2),  cx + (int) (sx / 2), sy)).copyTo(x_out_image_right);
            cv::hconcat(x_out_image_left, x_out_image_right, dst);
        }

        else if((cx + (int) (sx / 2)) > MAX_WIDTH)
        {
            img(cv::Rect(cx - (int) (sx / 2) - 1, cy - (int) (sy / 2), MAX_WIDTH - cx + (int) (sx / 2), sy)).copyTo(x_out_image_left);
            img(cv::Rect(0, cy - (int) (sy / 2), cx + (int) (sx / 2) - MAX_WIDTH, sy)).copyTo(x_out_image_right);
            cv::hconcat(x_out_image_left, x_out_image_right, dst);
        }

        else
        {
            sx--;
            img(cv::Rect(cx - (int) (sx / 2), cy - (int) (sy / 2), sx, sy)).copyTo(dst);
        }

        cv::resize(dst, rdst, cv::Size(roiwidth, roiheight), cv::INTER_LINEAR);
 
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
        res.alivetime = capture_time;
        res.depth = rdst.channels();
        res.height = rdst.rows;
        res.width = rdst.cols;
        res.roi = RoboCompCamera360RGB::TRoi{.xcenter=cx, .ycenter=cy, .xsize=sx, .ysize=sy, .finalxsize=res.width, .finalysize=res.height};
        std::cout << "TIMESTAMP: " << res.alivetime<< std::endl;
    }
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

