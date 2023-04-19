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

robosense::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
robosense::lidar::SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

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
    //	THE FOLLOWING IS JUST AN EXAMPLE
    //	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
    	try
    	{
    		RoboCompCommonBehavior::Parameter par = params.at("IPpc");
    		this->IP = par.value;
    	}
    	catch(const std::exception &e) { qFatal("Error reading config params"); }
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		param.input_type = robosense::lidar::InputType::ONLINE_LIDAR;
		param.input_param.host_address = "192.168.1.100"; // ip del pc que va a recibir los datos. El lidar se encuentra en la 192.168.1.200 (tiene api rest)
		//param.input_param.group_address = "192.168.1.200";
		param.input_param.msop_port = 6699;   ///< Set the lidar msop port number, the default is 6699
		param.input_param.difop_port = 7788;  ///< Set the lidar difop port number, the default is 7788
		param.lidar_type = robosense::lidar::LidarType::RSHELIOS;   ///< Set the lidar type. Make sure this type is correct
		param.print();
        driver.regPointCloudCallback(driverGetPointCloudFromCallerCallback, driverReturnPointCloudToCallerCallback); ///< Register the point cloud callback functions
		driver.regExceptionCallback(exceptionCallback);  ///< Register the exception callback function

		if (!driver.init(param))                         ///< Call the init function
		{
			std::cout << "Driver Initialize Error..." << std::endl;
			return;
		}
        driver.start();
        std::cout << "Driver initiated OK" << std::endl;

		timer.start(50);
	}
}

void SpecificWorker::compute()
{
   std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
    if (msg.get() == NULL)
        return;

    buffer_data.put(std::move(*msg), [](auto &&I, auto &T)
            {   T.resize(I.points.size()); int i=0;
                for(auto &&p: I.points) T[i++]=RoboCompLidar3D::TPoint{.x=p.x, .y=p.y, .z=p.z, .intensity=p.intensity};
            });

	//std::cout << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << std::endl;
    fps.print(std::to_string(msg->points.size()));
}


//
// @brief point cloud callback function. The caller should register it to the lidar driver.
//        Via this fucntion, the driver gets an free/unused point cloud message from the caller.
// @param msg  The free/unused point cloud message.
//
std::shared_ptr<PointCloudMsg> SpecificWorker::driverGetPointCloudFromCallerCallback(void)
{
    // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver,
    //       so please DO NOT do time-consuming task here.
    std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
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
void SpecificWorker::driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
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
void SpecificWorker::exceptionCallback(const robosense::lidar::Error& code)
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


RoboCompLidar3D::TLidarData SpecificWorker::Lidar3D_getLidarData()
{
    return buffer_data.get();
}



/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

