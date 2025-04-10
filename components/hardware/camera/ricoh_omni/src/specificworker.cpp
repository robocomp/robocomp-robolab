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

	if(this->startup_check_flag)
		this->startup_check();
	else
    {
        last_read.store(std::chrono::high_resolution_clock::now());

        if (pars.orin)
            pipeline = "thetauvcsrc mode=2K ! queue ! h264parse ! nvv4l2decoder ! nvvidconv ! queue ! videoconvert n-threads=2 ! video/x-raw,format=BGR ! queue ! appsink drop=true sync=false";
        else
            pipeline = "thetauvcsrc mode=2K ! queue ! h264parse ! nvdec ! gldownload ! queue ! videoconvert n-threads=2 ! video/x-raw,format=BGR ! queue ! appsink drop=true sync=false";
        //if (pars.orin)
        //pipeline = "thetauvcsrc mode=2K ! h264parse ! nvv4l2decoder ! nvvidconv ! videoconvert n-threads=2 ! video/x-raw,format=BGR ! appsink drop=true sync=false";
        //else
        //pipeline = "thetauvcsrc mode=2K ! h264parse ! nvdec ! gldownload ! videoconvert n-threads=2 ! video/x-raw,format=BGR ! appsink drop=true sync=false";

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
        this->Period = 33;
        timer.start(Period);
    }
}
void SpecificWorker::compute()
{
    /// check idle time
    if(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - last_read.load()).count() > MAX_INACTIVE_TIME)
    {
        fps.print("No requests in the last 5 seconds. Pausing. Comp wil continue in next call", 3000);
        return;
    }


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
            capture_time = image.timestamp;
            // self adjusting period to remote image source
            //self_adjust_period(image.period);
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
    if (!this->activated_camera)
        return res;

    last_read.store(std::chrono::high_resolution_clock::now());

    // Default values for invalid inputs
    sx = (sx <= 0) ? MAX_WIDTH : sx;
    sy = (sy <= 0) ? MAX_HEIGHT : sy;
    cx = (cx < 0) ? MAX_WIDTH / 2 : cx;
    cy = (cy < 0) ? MAX_HEIGHT / 2 : cy;
    roiwidth = (roiwidth < 0) ? MAX_WIDTH : roiwidth;
    roiheight = (roiheight < 0) ? MAX_HEIGHT : roiheight;

    // Get image from double buffer
    auto img = buffer_image.get_idemp();

    // Adjust ROI to stay within bounds
    int half_sx = sx / 2, half_sy = sy / 2;
    cy = std::clamp(cy, half_sy, MAX_HEIGHT - half_sy);
    cx = std::clamp(cx, half_sx, MAX_WIDTH - half_sx);

    cv::Mat roi;
    if (cx - half_sx < 0 || cx + half_sx > MAX_WIDTH) // Handle x out-of-bounds
    {
        cv::Mat left, right;
        if (cx - half_sx < 0)
        {
            img(cv::Rect(MAX_WIDTH - (half_sx - cx), cy - half_sy, half_sx - cx, sy)).copyTo(left);
            img(cv::Rect(0, cy - half_sy, cx + half_sx, sy)).copyTo(right);
        }
        else
        {
            img(cv::Rect(cx - half_sx, cy - half_sy, MAX_WIDTH - cx + half_sx, sy)).copyTo(left);
            img(cv::Rect(0, cy - half_sy, cx + half_sx - MAX_WIDTH, sy)).copyTo(right);
        }
        cv::hconcat(left, right, roi);
    }
    else
    {
        roi = img(cv::Rect(cx - half_sx, cy - half_sy, sx, sy));
    }

    // Resize only if necessary
    cv::Mat resized_roi;
    if (roiwidth != sx || roiheight != sy)
        cv::resize(roi, resized_roi, cv::Size(roiwidth, roiheight), cv::INTER_LINEAR);
    else
        resized_roi = roi;

    // Compress or copy image data
    if (pars.compressed)
    {
        std::vector<uchar> buffer;
        cv::imencode(".jpg", resized_roi, buffer, compression_params);
        res.image = std::move(buffer);
        res.compressed = true;
    }
    else
    {
        res.image.assign(resized_roi.data, resized_roi.data + (resized_roi.total() * resized_roi.elemSize()));
        res.compressed = false;
    }

    // Populate result metadata
    res.period = fps.get_period();
    res.timestamp = capture_time;
    res.depth = resized_roi.channels();
    res.height = resized_roi.rows;
    res.width = resized_roi.cols;
    res.roi = RoboCompCamera360RGB::TRoi{.xcenter = cx, .ycenter = cy, .xsize = sx, .ysize = sy, .finalxsize = res.width, .finalysize = res.height};

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

