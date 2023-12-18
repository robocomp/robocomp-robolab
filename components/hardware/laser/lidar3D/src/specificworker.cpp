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

robosense::lidar::SyncQueue <std::shared_ptr<PointCloudMsg>> free_cloud_queue;
robosense::lidar::SyncQueue <std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    this->startup_check_flag = startup_check;
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
    // Save locale setting
    const std::string oldLocale=std::setlocale(LC_NUMERIC,nullptr);
    // Force '.' as the radix point. If you comment this out,
    // you'll get output similar to the OP's GUI mode sample
    std::setlocale(LC_NUMERIC,"C");
    try
    {
        lidar_model = std::stoi(params.at("lidar_model").value);
        msop_port = std::stoi(params.at("msop_port").value);
        difop_port = std::stoi(params.at("difop_port").value);
        dest_pc_ip_addr = params.at("dest_pc_ip_addr").value;

        //Extrinsic
        float rx, ry, rz, tx, ty, tz;
        rx = std::stof(params["rx"].value);
        ry = std::stof(params["ry"].value);
        rz = std::stof(params["rz"].value);
        tx = std::stof(params["tx"].value);
        ty = std::stof(params["ty"].value);
        tz = std::stof(params["tz"].value);
        this->robot_lidar = Eigen::Translation3f(Eigen::Vector3f(tx,ty,tz));
        this->robot_lidar.rotate(Eigen::AngleAxisf (rx,Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf (ry, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ()));
        std::cout<<"Extrinsec Matrix:"<<std::endl<<this->robot_lidar.matrix()<<endl;

        //boundin box colision / hitbox
        float center_box_x, center_box_y, center_box_z, size_box_x, size_box_y, size_box_z;
        center_box_x = std::stof(params["center_box_x"].value);
        center_box_y = std::stof(params["center_box_y"].value);
        center_box_z = std::stof(params["center_box_z"].value);
        size_box_x = std::stof(params["size_box_x"].value);
        size_box_y = std::stof(params["size_box_y"].value);
        size_box_z = std::stof(params["size_box_z"].value);

        simulator = params["simulator"].value == "true";

        box_min.x() = center_box_x - size_box_x/2.0;//minx
        box_min.y() = center_box_y - size_box_y/2.0;//miny
        box_min.z() = center_box_z - size_box_z/2.0;//minz
        box_max.x() = center_box_x + size_box_x/2.0;//maxx
        box_max.y() = center_box_y + size_box_y/2.0;//maxy
        box_max.z() = center_box_z + size_box_z/2.0;//maxz

        floor_line = std::stof(params["floor_line"].value);

        std::cout<<"Hitbox min in millimetres:"<<std::endl<<this->box_min<<endl;
        std::cout<<"Hitbox max in millimetres:"<<std::endl<<this->box_max<<endl;
        std::cout<<"Floor line in millimetres:"<<std::endl<<this->floor_line<<endl;

    }catch (const std::exception &e)
    {std::cout <<"Error reading the config \n" << e.what() << std::endl << std::flush; }

    // Restore locale setting
    std::setlocale(LC_NUMERIC,oldLocale.c_str());

    if(simulator){
        dst_width = 1200;
        dst_height = 600;
    }
    else
    {
        dst_width = 1920;
        dst_height = 920;
    }
    
    return true;
}
void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;
    this->Period = 50;
    if (this->startup_check_flag)
    {
        this->startup_check();
    } else
    {
        if (!simulator)
        {
            param.input_type = robosense::lidar::InputType::ONLINE_LIDAR;
            param.input_param.host_address = dest_pc_ip_addr; // ip del pc que va a recibir los datos. El lidar se encuentra en la 192.168.1.200 (tiene api rest)
            //param.input_param.group_address = "192.168.50.111";
            param.input_param.msop_port = msop_port;   ///< Set the lidar msop port number, the default is 6699
            param.input_param.difop_port = difop_port;  ///< Set the lidar difop port number, the default is 7788
            param.lidar_type = lidar_model_list[lidar_model];   ///< Set the lidar type. Make sure this type is correct
            param.print();
            driver.regPointCloudCallback(driverGetPointCloudFromCallerCallback,
                                         driverReturnPointCloudToCallerCallback); ///< Register the point cloud callback functions
            driver.regExceptionCallback(exceptionCallback);  ///< Register the exception callback function}

            if (!driver.init(param))                         ///< Call the init function
            {
                std::cout << "Driver Initialize Error..." << std::endl;
                exit(-1);
            }
            driver.start();
            std::cout << "Driver initiated OK" << std::endl;
            
        }

        timer.start(this->Period);
        std::this_thread::sleep_for(std::chrono::milliseconds(this->Period*2));
        ready_to_go.store(true);
    }
}

void SpecificWorker::compute()
{
    RoboCompLidar3D::TData raw_lidar;
    int num_points = 0;
    try
    {
        if(simulator)
        {
            RoboCompLidar3D::TData raw_lidar_sim;
            if(lidar_model==1) //BPearl
                raw_lidar_sim = lidar3d_proxy->getLidarData("bpearl", 0, 360, 1);
            else //Helios
                raw_lidar_sim = lidar3d_proxy->getLidarData("helios", 0, 360, 1);

            raw_lidar = processLidarData(raw_lidar_sim);
            raw_lidar.timestamp = raw_lidar_sim.timestamp;
        }
        else
        {
            std::shared_ptr <PointCloudMsg> msg = stuffed_cloud_queue.popWait();
            if (msg.get() == NULL)
                return;
            
            raw_lidar = processLidarData(*msg);
            auto now = std::chrono::system_clock::now();
            raw_lidar.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        }
        raw_lidar.period = this->Period; // ms

        //Process lidar helios and add lidar data to doubleBuffer
        if (lidar_model==0)
        { //Helios
            auto processed_real_lidar_array = lidar2cam(raw_lidar);
            buffer_array_data.put(std::move(processed_real_lidar_array));
        }
        num_points = raw_lidar.points.size();
        buffer_data.put(std::move(raw_lidar));
    }
    catch (const Ice::Exception &e)
        {std::cout << __FUNCTION__ << " Error in Lidar Proxy" << std::endl;}

    fps.print("Num points: " + std::to_string(num_points) /*+ " Timestamp: " + std::to_string(raw_lidar.timestamp)*/);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//
// @brief point cloud callback function. The caller should register it to the lidar driver.
//        Via this function, the driver gets an free/unused point cloud message from the caller.
// @param msg  The free/unused point cloud message.
//
std::shared_ptr <PointCloudMsg> SpecificWorker::driverGetPointCloudFromCallerCallback(void)
{
    // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver,
    //       so please DO NOT do time-consuming task here.
    std::shared_ptr <PointCloudMsg> msg = free_cloud_queue.pop();
    
    if (msg.get() != NULL)
    {
        return msg;
    }

    return std::make_shared<PointCloudMsg>();
}
//
// @brief point cloud callback function. The caller should register it to the lidar driver.
//        Via this function, the driver gets/returns a stuffed point cloud message to the caller.
// @param msg  The stuffed point cloud message.
//
void SpecificWorker::driverReturnPointCloudToCallerCallback(std::shared_ptr <PointCloudMsg> msg)
{
    // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver,
    //       so please DO NOT do time-consuming task here. Instead, process it in caller's own thread. (see processCloud() below)
    stuffed_cloud_queue.clear();
    stuffed_cloud_queue.push(msg);
}
//
// @brief exception callback function. The caller should register it to the lidar driver.
//        Via this function, the driver inform the caller that something happens.
// @param code The error code to represent the error/warning/information
//
void SpecificWorker::exceptionCallback(const robosense::lidar::Error &code)
{
    // Note: This callback function runs in the packet-receving and packet-parsing/point-cloud_constructing thread of the driver,
    //       so please DO NOT do time-consuming task here.
    std::cout << code.toString() << std::endl;
}
inline bool SpecificWorker::isPointOutsideCube(const Eigen::Vector3f point, const Eigen::Vector3f box_min, const Eigen::Vector3f box_max) {
    return  (point.x() < box_min.x() || point.x() > box_max.x()) ||
            (point.y() < box_min.y() || point.y() > box_max.y()) ||
            (point.z() < box_min.z() || point.z() > box_max.z());
}
void SpecificWorker::self_adjust_period(int new_period)
{
    if(abs(new_period - this->Period) < 2)      // do it only if period changes
        return;
    if(new_period > this->Period)
    {
        this->Period += 1;
        timer.setInterval(this->Period);
    }
    else
    {
        this->Period -= 1;
        this->timer.setInterval(this->Period);
    }
}
std::optional<RoboCompLidar3D::TPoint> SpecificWorker::transform_filter_point(float x, float y, float z, int intensity)
{
    Eigen::Vector3f point(-y*1000, x*1000, z*1000); // convert to millimeters
    Eigen::Vector3f lidar_point = robot_lidar.linear() * point + robot_lidar.translation();

    if (isPointOutsideCube(lidar_point, box_min, box_max) and lidar_point.z() > floor_line)
    {
        auto distance2d = std::hypot(lidar_point.x(),lidar_point.y());
        auto r = lidar_point.norm();
        return RoboCompLidar3D::TPoint{.x=lidar_point.x(), .y=lidar_point.y(), .z=lidar_point.z(),
                .intensity=intensity, .phi=std::atan2(lidar_point.x(), lidar_point.y()),
                .theta=std::acos( lidar_point.z()/ r), .r=r, .distance2d=distance2d};
    }
    return {};
}
RoboCompLidar3D::TData SpecificWorker::processLidarData(const auto &input_points)
{
    RoboCompLidar3D::TData raw_lidar;
    if (!input_points.points.empty())
    {
        for (const auto &p : input_points.points)
        {
            auto transformed = transform_filter_point(p.x, p.y, p.z, p.intensity);
            if (transformed)
                raw_lidar.points.emplace_back(*transformed);
        }
        std::ranges::sort(raw_lidar.points, {}, &RoboCompLidar3D::TPoint::phi);
    }
    return raw_lidar;
}
std::vector<cv::Point2f> SpecificWorker::fish2equirect(const std::vector<cv::Point2f> &points)
{
    int aperture = M_PI; //Radians?
    int width = 3648;
    int height = 3648;

    int center_x = width / 2;
    int center_y = height / 2;

    std::vector<cv::Point2f> result;

    for (const auto& point : points) {
        double src_x_norm = (point.x - center_x) * 2.0 / width;
        double src_y_norm = -(point.y - center_y) * 2.0 / height;

        double r = std::sqrt(src_x_norm * src_x_norm + src_y_norm * src_y_norm);
        double p_x = src_x_norm;
        double p_z = src_y_norm;
        double p_y = (r != 0) ? r / std::tan(r * aperture / 2) : 0;

        double latitude = std::atan2(p_z, std::sqrt(p_x * p_x + p_y * p_y));
        double longitude = std::atan2(p_y, p_x);

        double dst_x_norm = longitude / M_PI;
        double dst_y_norm = latitude * 2.0 / M_PI;

        float x = static_cast<int>((-dst_x_norm / 2.0 + 0.5) * dst_width);
        float y = static_cast<int>((-dst_y_norm / 2.0 + 0.5) * dst_height);

        result.push_back(cv::Point2f{x,y});
    }
    return result;
}
RoboCompLidar3D::TDataImage SpecificWorker::lidar2cam (const RoboCompLidar3D::TData &lidar_data){
    std::vector<cv::Point3f> lidar_front, lidar_back;
    std::vector<cv::Point2f> lidar_front_2d, lidar_back_2d;
    RoboCompLidar3D::TData front, back;
    cv::Mat D;
    int resize_factor = 1.6;


    // Inicialización de rvec
    cv::Mat rvec;
    rvec.create(3, 1, CV_64F); // Crea una matriz de 3x1 con elementos de tipo double
    // Inicialización de tvec
    cv::Mat tvec;
    tvec.create(3, 1, CV_64F);
    // Inicialización de rvec_b
    cv::Mat rvec_b;
    rvec_b.create(3, 1, CV_64F);
    // Inicialización de tvec_b
    cv::Mat tvec_b;
    tvec_b.create(3, 1, CV_64F);
    cv::Mat K = (cv::Mat_<double>(3,3) << 1080, 0.0, 1824, 0.0, -1080, 1824, 0.0, 0.0, 1.0); // Ejemplo de inicialización

    //SIMULATOR & REAL CAMERA PARAMS
    if(simulator){
        rvec.at<double>(0, 0) = M_PI_2;
        rvec.at<double>(1, 0) = 0.0;
        rvec.at<double>(2, 0) = 0.0;
        tvec.at<double>(0, 0) = 0.0;
        tvec.at<double>(1, 0) = -1330.0;
        tvec.at<double>(2, 0) = -180.0;
        rvec_b.at<double>(0, 0) = M_PI_2;
        rvec_b.at<double>(1, 0) = 0.0;
        rvec_b.at<double>(2, 0) = 0.0;
        tvec_b.at<double>(0, 0) = 0.0;
        tvec_b.at<double>(1, 0) = -1330.0;
        tvec_b.at<double>(2, 0) = 180.0;
        resize_factor = 1;
        D = (cv::Mat_<double>(4,1) << 0.35308720224831864, -0.396793518453425, 0.20325216832157427, -0.03725173372715627); // Ejemplo de inicialización

        dst_width = 1200;
        dst_height = 600;
    }
    else
    {
        rvec.at<double>(0, 0) = M_PI_2;
        rvec.at<double>(1, 0) = 0.0;
        rvec.at<double>(2, 0) = 0.0;
        tvec.at<double>(0, 0) = 0.0;
        tvec.at<double>(1, 0) = -1330.0;
        tvec.at<double>(2, 0) = 170.0;
        rvec_b.at<double>(0, 0) = M_PI_2;
        rvec_b.at<double>(1, 0) = 0.0;
        rvec_b.at<double>(2, 0) = 0.0;
        tvec_b.at<double>(0, 0) = 0.0;
        tvec_b.at<double>(1, 0) = -1330.0;
        tvec_b.at<double>(2, 0) = -170.0;
        D = (cv::Mat_<double>(4,1) << 0.35208720224831864, -0.396793518453425, 0.20325216832157427, -0.03725173372715627); // Ejemplo de inicialización
        resize_factor = 1.6;
    }

    //SPLIT FRONT/BACK POINTS
    for (auto &p : lidar_data.points)
    {
        if(p.y > tvec_b.at<double>(2,0))
        {
            lidar_front.push_back(cv::Point3f(p.x, p.y, -p.z));
            front.points.push_back(p);
        }
        else
        {
            // Invert the x, y coordinates
            lidar_back.push_back(cv::Point3f(-p.x, -p.y, -p.z));
            back.points.push_back(p);
        }
    }

    //PROJECT POINTS 3D -> 2D FISHEYE
    cv::fisheye::projectPoints(lidar_front, lidar_front_2d, rvec, tvec, K, D, 0);
    cv::fisheye::projectPoints(lidar_back, lidar_back_2d, rvec_b, tvec_b, K, D, 0);

    //PROJECT POINTS 2D FISHEYE -> 2D EQUIRECTANGULAR
    lidar_front_2d = fish2equirect(lidar_front_2d);
    lidar_back_2d = fish2equirect(lidar_back_2d);

    //FRAME
    cv::Mat cv_frame(cv::Size(dst_width, dst_height), CV_32FC3, cv::Scalar(0,0,0));

    // Structure vectors
    RoboCompLidar3D::TDataImage data_image;
    data_image.timestamp = lidar_data.timestamp;
    std::vector<float> x; x.reserve(front.points.size() + back.points.size());
    std::vector<float> y; y.reserve(front.points.size() + back.points.size());
    std::vector<float> z; z.reserve(front.points.size() + back.points.size());
    std::vector<int> x_p; x_p.reserve(front.points.size() + back.points.size());
    std::vector<int> y_p; y_p.reserve(front.points.size() + back.points.size());

    // COMPOSE COMPLETE IMAGE, CENTER FRONT IMAGE AND SPLIT BACK IMAGE
    for(auto&& [l, p] : iter::zip(front.points, lidar_front_2d))
    {
        l.pixelX = (p.x + dst_width / 4) * resize_factor;
        l.pixelY = p.y * resize_factor;
        x.push_back(l.x);
        y.push_back(l.y);
        z.push_back(l.z);
        x_p.push_back(l.pixelX);
        y_p.push_back(l.pixelY);
        cv::Vec3f& pixel = cv_frame.at<cv::Vec3f>(l.pixelY, l.pixelX);
        pixel[0] = l.x;
        pixel[1] = l.y;
        pixel[2] = l.z;
    }

    for(auto&& [l, p] : iter::zip(back.points, lidar_back_2d))
    {
        if(p.x < dst_width/4) {
            l.pixelX = (p.x + dst_width * 3 / 4) * resize_factor;
        }
        else{
            l.pixelX = (p.x - dst_width/4) * resize_factor;
        }

        l.pixelY = p.y * resize_factor;

        cv::Vec3f& pixel = cv_frame.at<cv::Vec3f>(l.pixelY, l.pixelX);
        pixel[0] = l.x;
        pixel[1] = l.y;
        pixel[2] = l.z;
        x.push_back(l.x);
        y.push_back(l.y);
        z.push_back(l.z);
        x_p.push_back(l.pixelX);
        y_p.push_back(l.pixelY);
    }

//    cv::resize(cv_frame, cv_frame, cv::Size(640,320),cv::INTER_NEAREST);
//    cv::imshow("X",cv_frame);
    // cv::waitKey(1);
    data_image.XArray = x;
    data_image.YArray = y;
    data_image.ZArray = z;
    data_image.XPixel = x_p;
    data_image.YPixel = y_p;
    return  data_image;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////// Interfaces                                                            //////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 @brief
 @param name - name of the lidar
 @param start - start angle in radians
 @param len - length of the angle in radians
 @param decimationDegreeFactor - factor of reduction in degrees 
*/
RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarData(std::string name, float start, float len, int decimationDegreeFactor)
{
    

     //LiDAR not started
    if(not ready_to_go)
        return {};

    RoboCompLidar3D::TData buffer = buffer_data.get_idemp();

    // Check for nominal conditions
    if(len >= 2*M_PI  and decimationDegreeFactor == 1)
        return buffer;

    RoboCompLidar3D::TPoints filtered_points;

    //Get all LiDAR
    if (len >= 2*M_PI)
        filtered_points = std::move(buffer.points);
        //Cut range LiDAR
    else
    {
        //Start and end angles
        auto rad_start = start;
        auto rad_end = start + len;

        //Start Iterator, this is the end if there are surpluses, otherwise it will be modified by the defined end.
        auto it_begin = std::find_if(buffer.points.begin(), buffer.points.end(),
                                     [_start=rad_start](const RoboCompLidar3D::TPoint& point)
                                     {return _start < point.phi;});
        //End Iterator
        auto it_end = buffer.points.end();
        //The clipping exceeds pi, we assign the excess to the result
        if (rad_end > M_PI)
            filtered_points.assign(std::make_move_iterator(buffer.points.begin()), 
                                    std::make_move_iterator(std::find_if(buffer.points.begin(), buffer.points.end(),
                                                            [_end=rad_end - 2*M_PI](const RoboCompLidar3D::TPoint& point)
                                                            {return _end < point.phi;})));
        else
            it_end = std::find_if(it_begin, buffer.points.end(),
                                  [_end=rad_end](const RoboCompLidar3D::TPoint& point)
                                  {return _end < point.phi;});
        //we insert the cut with 2PI limit
        filtered_points.insert(filtered_points.end(), std::make_move_iterator(it_begin), std::make_move_iterator(it_end));
    }

    //Apply decimal factor reduction
    if (decimationDegreeFactor != 1){
        //Decimal factor calculation
        float rad_factor = qDegreesToRadians((float)decimationDegreeFactor);
        float tolerance = qDegreesToRadians(0.5);

        //We remove the points that are of no interest
        filtered_points.erase(std::remove_if(filtered_points.begin(), filtered_points.end(),
                                            [rad_factor, tolerance](const RoboCompLidar3D::TPoint& point)
                                            {float remainder = abs(fmod(point.phi, rad_factor));
                                                return !(remainder <= tolerance || remainder >= rad_factor - tolerance);
                                            }), filtered_points.end());
    }
    return RoboCompLidar3D::TData {.points=filtered_points, .period=buffer.period, .timestamp=buffer.timestamp};
}
/*
 @brief
 @param name - name of the lidar
 @param distance - maximum distance view of the lidar
 @param decimationDegreeFactor - factor of reduction in degrees 

*/
RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor)
{
    //LiDAR not started
    if(not ready_to_go)
        return {};

    //Get LiDAR data
    RoboCompLidar3D::TData buffer = buffer_data.get_idemp();

    //Sort distance and cut
    std::ranges::sort(buffer.points, {}, &RoboCompLidar3D::TPoint::distance2d);
    RoboCompLidar3D::TPoints filtered_points(std::make_move_iterator(buffer.points.begin()), std::make_move_iterator(
            std::find_if(buffer.points.begin(), buffer.points.end(),
                         [_distance=distance](const RoboCompLidar3D::TPoint& point)
                         {return _distance < point.distance2d;})));
    //ReSort by phi
    std::ranges::sort(buffer.points, {}, &RoboCompLidar3D::TPoint::phi);

    //Apply decimal factor reduction
    if (decimationDegreeFactor != 1){
        //Decimal factor calculation
        float rad_factor = qDegreesToRadians((float)decimationDegreeFactor);
        float tolerance = qDegreesToRadians(0.5);

        //We remove the points that are of no interest
        filtered_points.erase(std::remove_if(filtered_points.begin(), filtered_points.end(),
                                            [rad_factor, tolerance](const RoboCompLidar3D::TPoint& point)
                                            {float remainder = abs(fmod(point.phi, rad_factor));
                                                return !(remainder <= tolerance || remainder >= rad_factor - tolerance);
                                            }), filtered_points.end());
    }
    return RoboCompLidar3D::TData {.points=filtered_points, .period=buffer.period, .timestamp=buffer.timestamp};

}
/*
 @brief
 @param name - name of the lidar
*/
RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataProyectedInImage(std::string name)
{
    //LiDAR not started
    if (not ready_to_go)
        return {};

    RoboCompLidar3D::TData buffer;
    RoboCompLidar3D::TPoints processed_points;

    //Get LiDAR data
    return buffer_data.get_idemp();
}
RoboCompLidar3D::TDataImage SpecificWorker::Lidar3D_getLidarDataArrayProyectedInImage(std::string name)
{
    RoboCompLidar3D::TDataImage ret;
    //LiDAR not started
    if(not ready_to_go)
        return {};
    
    return buffer_array_data.get_idemp();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}
