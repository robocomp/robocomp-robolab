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
        display_rgb = (params.at("display_rgb").value == "true") or (params.at("display_rgb").value == "True");
        align_frames = (params["align_frames"].value == "true") or (params["align_frames"].value == "True");
        display_depth = (params["display_depth"].value == "true") or (params["display_depth"].value == "True");
        img_width = std::stoi(params["img_width"].value);
        img_height = std::stoi(params["img_height"].value);
        if(img_width < 0 or img_width > 1500 or img_height < 0 or img_height > 1500)
        { qWarning() << "Illegal image size. Terminating"; std::terminate();}
        img_freq = std::stoi(params["fps"].value);
        if(img_freq < 0 or img_freq > 200)
        { qWarning() << "Illegal fps value. Terminating"; std::terminate();}
        serial_number = params["serial_number"].value;

        display_compressed = (params["display_compressed"].value == "true") or (params["display_compressed"].value == "True");
        std::cout << std::boolalpha << "Config params: display_rgb " << display_rgb <<
                                       " display_depth " << display_depth <<
                                       " display_compressed " << display_compressed <<
                                       " align_frames " << align_frames <<
                                       " fps " << img_freq <<
                                       " serial_number " << serial_number <<
                                       std::endl;
    }
    catch (const std::exception &e){std::cout << e.what() << std::endl;}
    return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;
    this->Period = 1;
    if (this->startup_check_flag)
        this->startup_check();
    else
    {
        compression_params_image.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params_image.push_back(50);
        compression_params_depth.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params_depth.push_back(100);

        //compression_params_image.push_back(cv::IMWRITE_PNG_COMPRESSION);//PNG
        //compression_params_depth.push_back(cv::IMWRITE_PNG_COMPRESSION);
        //compression_params_image.push_back(9);
        //compression_params_depth.push_back(9);

        try
        {
            rs2::context context;
            auto devices = context.query_devices();
            std::set<std::string> cameras;
            std::cout << "Detected cameras:" << std::endl;
            for (const auto dev: devices)
            {
                std::cout << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
                cameras.insert(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
            }
            if(not cameras.contains(serial_number))
            {
                std::cout << "Unspecified serial number or unavailable camera with number: " << serial_number << std::endl;
                serial_number = *cameras.begin();
                std::cout << "Selecting the first one detected: " << serial_number <<
                             " at port: " << devices.front().get_info(RS2_CAMERA_INFO_PHYSICAL_PORT) << std::endl;
            }
            else
            {
                for(const auto d : devices)
                    if(d.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == serial_number)
                        std::cout << "Opening camera: " << serial_number << " at port: " << d.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT) << std::endl;
            }
            cfg.enable_device(serial_number);
            cfg.enable_stream(RS2_STREAM_DEPTH, img_width, img_height, RS2_FORMAT_Z16, img_freq);
            cfg.enable_stream(RS2_STREAM_COLOR, img_width, img_height, RS2_FORMAT_BGR8, img_freq);
            profile = pipe.start(cfg);

            // Each depth camera might have different units for depth pixels, so we get it here
            // Using the pipeline's profile, we can retrieve the device that the pipeline uses
            depth_scale = get_depth_scale(profile.get_device());

            // Create a rs2::align object.
            //align_to_rgb = std::make_unique<rs2::align>(RS2_STREAM_COLOR);
            //align_to_depth = std::make_unique<rs2::align>(RS2_STREAM_DEPTH);

            // Define a variable for controlling the distance to clip
            //float depth_clipping_distance = 10000.f;

            cam_intr = pipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
            depth_intr = pipe.get_active_profile().get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();

            qInfo() << __FUNCTION__ << "    Camera opened with img size: [" << cam_intr.width << cam_intr.height << "] and FPS:" << img_freq;

            filters.emplace_back("Decimate", dec_filter);
            // filters.emplace_back(disparity_filter_name, depth_to_disparity);
            filters.emplace_back("Spatial", spat_filter);
            filters.emplace_back("Temporal", temp_filter);
            filters.emplace_back("HFilling", holef_filter);
        }
        catch (std::exception &e)
        { std::cout << e.what() << std::endl; }

        timer.start(this->Period);
    }
}

void SpecificWorker::compute()
{
    rs2::frameset frameset = pipe.wait_for_frames();
    auto now = duration_cast< milliseconds >(system_clock::now().time_since_epoch()).count();
    const auto &color = frameset.get_color_frame();
    auto depth = frameset.get_depth_frame();

    //If one of them is unavailable, continue iteration
    if (not depth or not color)
        return;

    for (auto &&filter : filters)
        if (filter.is_enabled)
            depth = filter.filter.process(depth);

    if (display_rgb)
    {
        cv::Mat frame(cv::Size(cam_intr.width, cam_intr.height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        cv::imshow("RGB image", frame);
        cv::waitKey(1);
    }

    if (display_depth)
    {
        rs2::frame depth_color = depth.apply_filter(color_map);
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();
        cv::Mat frame_depth(cv::Size(w, h), CV_8UC3, (void*)depth_color.get_data(), cv::Mat::AUTO_STEP);
        cv::imshow("DEPTH image", frame_depth);
        cv::waitKey(1);
    }

    rgb_frame_write = std::make_tuple(color, now);
    depth_frame_write = std::make_tuple(depth, now);
    swap_mutex.lock();
        rgb_frame_write.swap(rgb_frame_read);
        depth_frame_write.swap(depth_frame_read);
    swap_mutex.unlock();

    fps.print("FPS:");
    ready_to_go = true;
}

float SpecificWorker::get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            //qInfo() << "Depth scale: " << dpt.get_depth_scale();
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}


//////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}
////////////////////////////////////////////////////////////////////////////
RoboCompCameraRGBDSimple::TRGBD SpecificWorker::CameraRGBDSimple_getAll(std::string camera)
{
    RoboCompCameraRGBDSimple::TRGBD rgbd;
    const std::lock_guard<std::mutex> lg(swap_mutex);
    auto &[rgb_, timestamp] = rgb_frame_read;
    auto rgb = rgb_.as<rs2::video_frame>();
    int width = rgb.get_width();
    int height = rgb.get_height();
    int rgb_bpp = rgb.get_bytes_per_pixel();
    rgbd.image.depth = rgb_bpp;
    rgbd.image.width = width;
    rgbd.image.height = height;
    rgbd.image.cameraID = 0;
    rgbd.image.focalx = cam_intr.fx;
    rgbd.image.focaly = cam_intr.fy;
    rgbd.image.alivetime = timestamp;
    rgbd.image.period = fps.get_period();
    rgbd.image.compressed = false;
    if (display_compressed)
    {
        rgbd.image.cameraID = 0;
        rgbd.image.focalx = depth_intr.fx;
        rgbd.image.focaly = depth_intr.fy;
        rgbd.image.alivetime = 0;
        rgbd.image.compressed = true;
        cv::Mat frame(cv::Size(rgbd.image.width, rgbd.image.height), CV_8UC3, &rgbd.image.image[0], cv::Mat::AUTO_STEP);
        cv::imencode(".png", frame, buffer, compression_params_image);
        std::cout << "raw: " << frame.total() * frame.elemSize() << " compressed: " << buffer.size() << " Ratio:"
                  << frame.total() * frame.elemSize() / buffer.size() << std::endl;
        rgbd.image.image.assign(buffer.begin(), buffer.end());
    }
    else
    {
        uint8_t* byte_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(rgb.get_data()));
        rgbd.image.image.insert(rgbd.image.image.end(),byte_frame, byte_frame+width*height*rgb_bpp);
    }

    // Depth
    //auto frame = depth_frame_read.as<rs2::depth_frame>();
    auto &[frame_, dtimestamp] = depth_frame_read;
    auto frame = frame_.as<rs2::video_frame>();
    int dwidth = frame.get_width();
    int dheight = frame.get_height();
    rgbd.depth.width = dwidth;
    rgbd.depth.height = dheight;
    rgbd.depth.cameraID = 0;
    rgbd.depth.focalx = depth_intr.fx;
    rgbd.depth.focaly = depth_intr.fy;
    rgbd.depth.alivetime = dtimestamp;
    rgbd.image.period = fps.get_period();
    rgbd.depth.compressed = false;
    if (display_compressed)
    {
        rgbd.depth.cameraID = 0;
        rgbd. depth.focalx = depth_intr.fx;
        rgbd.depth.focaly = depth_intr.fy;
        rgbd.depth.alivetime = 0;
        rgbd.depth.depthFactor = depth_scale;
        rgbd.depth.compressed = true;
        cv::Mat frame_depth(cv::Size(rgbd.depth.width, rgbd.depth.height), CV_32F, &rgbd.depth.depth[0],
                            cv::Mat::AUTO_STEP);
        cv::imencode(".jpg", frame_depth, buffer, compression_params_depth);// modificar ".jpg" a ".png" ty viceversa, tipo de compresión
        std::cout << "raw: " << frame_depth.total() * frame_depth.elemSize() << " compressed: " << buffer.size()
                  << " Ratio:" << frame_depth.total() * frame_depth.elemSize() / buffer.size() << std::endl;
        rgbd.depth.depth.assign(buffer.begin(), buffer.end());
    }
    else
    {
        rgbd.depth.depth.resize(dwidth*dheight*sizeof(float));
        const uint16_t* byte_frame = reinterpret_cast<const uint16_t*>(frame.get_data());
        float real_depth;
        std::size_t k = 0;
        for(int i=0; i<dwidth*dheight; i++)
        {
            real_depth = depth_scale * byte_frame[i];
            const std::uint8_t *d = reinterpret_cast<uint8_t const *>(&real_depth);
            rgbd.depth.depth[k] = d[0];
            rgbd.depth.depth[k+1] = d[1];
            rgbd.depth.depth[k+2] = d[2];
            rgbd.depth.depth[k+3] = d[3];
            k += 4;
        }
    }
    return rgbd;
}

RoboCompCameraRGBDSimple::TDepth SpecificWorker::CameraRGBDSimple_getDepth(std::string camera)
{
    RoboCompCameraRGBDSimple::TDepth depth;
    if(not ready_to_go) return depth;
    const std::lock_guard<std::mutex> lg(swap_mutex);
    auto &[frame_, timestamp] = depth_frame_read;
    auto frame = frame_.as<rs2::video_frame>();
    int width = frame.get_width();
    int height = frame.get_height();
    depth.width = width;
    depth.height = height;
    depth.cameraID = 0;
    depth.focalx = depth_intr.fx;
    depth.focaly = depth_intr.fy;
    depth.alivetime = timestamp;
    depth.period = fps.get_period();
    depth.compressed = false;
    if (display_compressed)
    {
        depth.cameraID = 0;
        depth.focalx = depth_intr.fx;
        depth.focaly = depth_intr.fy;
        depth.alivetime = timestamp;
        depth.depthFactor = depth_scale;
        depth.compressed = true;
        cv::Mat frame_depth(cv::Size(rgbd.depth.width, rgbd.depth.height), CV_32F, &rgbd.depth.depth[0],
                            cv::Mat::AUTO_STEP);
        cv::imencode(".jpg", frame_depth, buffer, compression_params_depth);// modificar ".jpg" a ".png" ty viceversa, tipo de compresión
        std::cout << "raw: " << frame_depth.total() * frame_depth.elemSize() << " compressed: " << buffer.size()
                  << " Ratio:" << frame_depth.total() * frame_depth.elemSize() / buffer.size() << std::endl;
        rgbd.depth.depth.assign(buffer.begin(), buffer.end());
        return rgbd.depth;
    }
    else
    {
        depth.depth.resize(width*height*sizeof(float));
        const uint16_t* byte_frame = reinterpret_cast<const uint16_t*>(frame.get_data());
        float real_depth;
        std::size_t k = 0;
        for(int i=0; i<width*height; i++)
        {
            real_depth = depth_scale * byte_frame[i];
            const std::uint8_t *d = reinterpret_cast<uint8_t const *>(&real_depth);
            depth.depth[k] = d[0];
            depth.depth[k+1] = d[1];
            depth.depth[k+2] = d[2];
            depth.depth[k+3] = d[3];
            k += 4;
        }
        return depth;
    }
}

RoboCompCameraRGBDSimple::TImage SpecificWorker::CameraRGBDSimple_getImage(std::string camera)
{
    RoboCompCameraRGBDSimple::TImage image;
    if(not ready_to_go) return image;
    const std::lock_guard<std::mutex> lg(swap_mutex);
    auto &[rgb_, timestamp] = rgb_frame_read;
    auto rgb = rgb_.as<rs2::video_frame>();
    int width = rgb.get_width();
    int height = rgb.get_height();
    int rgb_bpp = rgb.get_bytes_per_pixel();
    image.depth = rgb_bpp;
    image.width = width;
    image.height = height;
    image.cameraID = 0;
    image.focalx = cam_intr.fx;
    image.focaly = cam_intr.fy;
    image.alivetime = timestamp;
    image.period = fps.get_period();
    image.compressed = false;
    if (display_compressed)
    {
        image.cameraID = 0;
        image.focalx = depth_intr.fx;
        image.focaly = depth_intr.fy;
        image.alivetime = 0;
        image.compressed = true;
        cv::Mat frame(cv::Size(rgbd.image.width, rgbd.image.height), CV_8UC3, &rgbd.image.image[0], cv::Mat::AUTO_STEP);
        cv::imencode(".png", frame, buffer, compression_params_image);
        std::cout << "raw: " << frame.total() * frame.elemSize() << " compressed: " << buffer.size() << " Ratio:"
                  << frame.total() * frame.elemSize() / buffer.size() << std::endl;
        image.image.assign(buffer.begin(), buffer.end());
        return rgbd.image;
    }
    else
    {
        uint8_t* byte_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(rgb.get_data()));
        image.image.insert(image.image.end(),byte_frame, byte_frame+width*height*rgb_bpp);
        return image;
    }
}

RoboCompCameraRGBDSimple::TPoints SpecificWorker::CameraRGBDSimple_getPoints(std::string camera)
{
	return RoboCompCameraRGBDSimple::TPoints();

}

/**************************************/
// From the RoboCompCameraRGBDSimplePub you can publish calling this methods:
// this->camerargbdsimplepub_pubproxy->pushRGBD(...)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

// optionally align depth frame to rgb frame. FPS drops to 6
//if(align_frames)
//    frameset = align_to_rgb->process(frameset);
