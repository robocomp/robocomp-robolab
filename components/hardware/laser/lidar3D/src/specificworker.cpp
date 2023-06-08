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
    try
    {
        lidar_model = std::stoi(params.at("lidar_model").value);
        msop_port = std::stoi(params.at("msop_port").value);
        difop_port = std::stoi(params.at("difop_port").value);
        dest_pc_ip_addr = params.at("dest_pc_ip_addr").value;
        simulator = params["simulator"].value == "true";
    }
    catch (const std::exception &e)
    { std::cout << e.what() << std::endl; }

    return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;
    this->Period = period;
    if (this->startup_check_flag)
    {
        this->startup_check();
    } else
    {
        if (not simulator)
        {
            param.input_type = robosense::lidar::InputType::ONLINE_LIDAR;
            param.input_param.host_address = dest_pc_ip_addr; // ip del pc que va a recibir los datos. El lidar se encuentra en la 192.168.1.200 (tiene api rest)
            //param.input_param.group_address = "192.168.1.200";
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
        }
        timer.start(50);
    }
}

void SpecificWorker::compute()
{
    if (not simulator)  // REAL LIDAR
    {
        std::shared_ptr <PointCloudMsg> msg = stuffed_cloud_queue.popWait();
        if (msg.get() == NULL)
            return;

        buffer_data.put(std::move(*msg), [](auto &&I, auto &T)
        {
            cout << I.points.size() << endl;
            if (I.points.size() == 28800)
            {
                T.resize(I.points.size());
                int i = 0;

                for (auto &&p: I.points) T[i++] = RoboCompLidar3D::TPoint{.x=-p.y, .y=p.x, .z=p.z, .intensity=p.intensity};
            }
        });

        //std::cout << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << std::endl;
        fps.print(std::to_string(msg->points.size()));
    }
    else
        fps.print("Connected to simulator");
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


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}


//RoboCompLidar3D::TLidarData SpecificWorker::Lidar3D_getLidarData(int start, int len)
//{
//    RoboCompLidar3D::TLidarData data;
//    if (coppelia==false)
//    {
//                auto buffer = buffer_data.get();
//                //auto start = start;
//                //auto leng = 180;
//                auto eje_start = (start / 0.4) * 32;
//                cout << "EJE START: " << eje_start << std::endl;
//                cout << buffer[eje_start].x << std::endl;
//                auto eje_leng = (len / 0.4) * 32;
//                auto total = (360 / 0.4) * 32;
////    auto init = buffer_data.get()[eje_start];
//
//                for (auto i = 0; i < eje_leng; i++) {
//                    data.push_back(
//                            RoboCompLidar3D::TPoint{.x=buffer[eje_start].x, .y=buffer[eje_start].y, .z=buffer[eje_start].z, .intensity=buffer[eje_start].intensity});
//                    eje_start = 1 + eje_start;
//                    if (fmod((eje_start), 28800) == 0) {
//                        eje_start = 1;
//                    }
//
//                }
//        }
//    else
//    {
//        double start_angle = remap_angle(start - 180);
//        double end_angle = remap_angle(len - 180);
//        data = this->lidar3d_proxy->getLidarData(static_cast<int>(start_angle), static_cast<int>(end_angle));
//    }
//    return data;
//
//}


RoboCompLidar3D::TLidarData SpecificWorker::Lidar3D_getLidarData(int start, int len)
{
    RoboCompLidar3D::TLidarData data;

    const int FACTOR = 80;  // pre-calculate this: (1 / 0.4) * 32

    if (not simulator)
    {
        auto buffer = buffer_data.get();
        int start_angle = remap_angle_real(start);
        auto eje_start = start_angle * FACTOR;
        auto eje_leng = len * FACTOR;

        data.reserve(eje_leng);  // pre-allocate memory

        for (int i = 0; i < eje_leng; ++i)
        {
            data.push_back(RoboCompLidar3D::TPoint{.x=buffer[eje_start].x * 1000, .y=buffer[eje_start].y * 1000, .z=
            buffer[eje_start].z * 1000, .intensity=buffer[eje_start].intensity});
            eje_start++;

            if (eje_start % 28800 == 0)
                eje_start = 1;
        }
    }
    else
    {
        double start_angle = remap_angle(start);
        double len_angle = remap_angle(len);
        data = this->lidar3d_proxy->getLidarData(static_cast<int>(start_angle), static_cast<int>(len_angle));
    }

    return data;
}


/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData



/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
