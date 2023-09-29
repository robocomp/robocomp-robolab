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
#include "cppitertools/slice.hpp"
#include "math.h"

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
            return;
        }
        driver.start();
        std::cout << "Driver initiated OK" << std::endl;

        timer.start(this->Period);
        sleep(this->Period/50); //20*period, wait to execution compute
        ready_to_go.store(true);
    }
}

void SpecificWorker::compute()
{
    std::shared_ptr <PointCloudMsg> msg = stuffed_cloud_queue.popWait();
    if (msg.get() == NULL)
        return;
    auto lidar_size = msg->points.size();
    buffer_real_data.put(std::move(*msg), [this](auto &&I, auto &T)
    {
        //cout << I.points.size() << endl;
        if (I.points.size() > 0 )
        {
            T.points.resize(I.points.size());
            int i = 0;
            for (auto &&p: I.points)
            {
                Eigen::Vector3f point(-p.y*1000, p.x*1000, p.z*1000);
                Eigen::Vector3f lidar_point = robot_lidar.linear() * point + robot_lidar.translation();
                
                if (isPointOutsideCube(lidar_point, box_min, box_max) and lidar_point.z() > floor_line)
                {
                    auto distance2d = std::hypot(lidar_point.x(),lidar_point.y());
                    auto r = lidar_point.norm();
                    T.points[i++] = RoboCompLidar3D::TPoint{.x=lidar_point.x(), .y=lidar_point.y(), .z=lidar_point.z(), 
                                        .intensity=p.intensity, .phi=std::atan2(-lidar_point.x(), lidar_point.y()), 
                                        .theta=std::acos( lidar_point.z()/ r), .r=r, .distance2d=distance2d};
                }
            }
            T.points.resize(i);
            std::ranges::sort(T.points, {}, &RoboCompLidar3D::TPoint::phi);
        }
    });
    fps.print("Connected to real device, " + std::to_string(lidar_size) + " points");
}

//
// @brief point cloud callback function. The caller should register it to the lidar driver.
//        Via this fucntion, the driver gets an free/unused point cloud message from the caller.
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

double SpecificWorker::remap_angle(double angle)
{
    if (angle >= 0)
        return 2.5 * angle;
    else
        return 900 + 2.5 * angle;
}

int SpecificWorker::remap_angle_real(int angle)
{
    return (360 - (angle - 180)) % 360;
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
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////// Interfaces                                                            //////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

void SpecificWorker::self_adjust_period(int new_period)
{
    if(abs(new_period - this->Period) < 2)      // do it only if period changes
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarData(std::string name, float start, float len, int decimationDegreeFactor)
{
    //LiDAR not started
    if(not ready_to_go)
        return {};
    #if DEBUG
        auto cstart = std::chrono::high_resolution_clock::now();
    #endif

    //Get LiDAR data
    RoboCompLidar3D::TData buffer = buffer_real_data.get_idemp();

    // Check for nominal conditions
    if(len == 360  and decimationDegreeFactor == 1)
        return buffer;

    RoboCompLidar3D::TPoints filtered_points; 
    //Get all LiDAR
    if (len == 360)
        filtered_points = std::move(buffer.points);
    //Cut range LiDAR
    else{
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
            filtered_points.assign(std::make_move_iterator(buffer.points.begin()), std::make_move_iterator(std::find_if(buffer.points.begin(), buffer.points.end(),
                        [_end=rad_end - 2*M_PI](const RoboCompLidar3D::TPoint& point) 
                        {return _end < point.phi;})));
        else 
            it_end = std::find_if(it_begin, buffer.points.end(),
                        [_end=rad_end](const RoboCompLidar3D::TPoint& point)
                        {return _end < point.phi;});
        //we insert the cut with 2PI limit
        filtered_points.insert(filtered_points.end(), std::make_move_iterator(it_begin), std::make_move_iterator(it_end));
        #if DEBUG
            std::cout << "Time prepare lidar: " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - cstart).count() << " microseconds" << std::endl<<std::flush;
        #endif
    }
    //
    if (decimationDegreeFactor == 1)
        return RoboCompLidar3D::TData {filtered_points, buffer.period, buffer.timestamp};
    #if DEBUG   
        cstart = std::chrono::high_resolution_clock::now();
    #endif

    //Decimal factor calculation
    float rad_factor = qDegreesToRadians((float)decimationDegreeFactor);
    float tolerance = qDegreesToRadians(0.5);

    //We remove the points that are of no interest 
    filtered_points.erase(std::remove_if(filtered_points.begin(), filtered_points.end(),
            [rad_factor, tolerance](const RoboCompLidar3D::TPoint& point) 
            {float remainder = fmod(point.phi, rad_factor);
                return !(remainder <= tolerance || remainder >= rad_factor - tolerance);
            }), filtered_points.end());
    #if DEBUG
        std::cout << "Time for cut lidar: " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - cstart).count() << " microseconds" << std::endl<<std::flush;
    #endif
    return RoboCompLidar3D::TData {filtered_points, buffer.period, buffer.timestamp};
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance)
{
    //LiDAR not started
    if(not ready_to_go)
        return {};

    #if DEBUG
        auto cstart = std::chrono::high_resolution_clock::now();
    #endif
    //Get LiDAR data
    RoboCompLidar3D::TData buffer = buffer_real_data.get_idemp();

    #if DEBUG
        std::cout << "Time get buffer lidar: " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - cstart).count() << " microseconds" << std::endl<<std::flush;
        cstart = std::chrono::high_resolution_clock::now();
    #endif
    std::ranges::sort(buffer.points, {}, &RoboCompLidar3D::TPoint::distance2d);
    RoboCompLidar3D::TPoints filtered_points(std::make_move_iterator(buffer.points.begin()), std::make_move_iterator(
        std::find_if(buffer.points.begin(), buffer.points.end(),
                [_distance=distance](const RoboCompLidar3D::TPoint& point) 
                {return _distance < point.distance2d;})));
    std::ranges::sort(buffer.points, {}, &RoboCompLidar3D::TPoint::phi);
    

    #if DEBUG
        std::cout << "Time filter lidar: " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - cstart).count() << " microseconds" << std::endl<<std::flush;
    #endif
    return RoboCompLidar3D::TData {filtered_points, buffer.period, buffer.timestamp};
}

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData