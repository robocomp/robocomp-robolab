/*
 *    Copyright (C) 2024 by YOUR NAME HERE
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
#include <cppitertools/enumerate.hpp>

robosense::lidar::SyncQueue <std::shared_ptr<PointCloudMsg>> free_cloud_queue;
robosense::lidar::SyncQueue <std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif


		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period,
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}

	}
}
SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
}
void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;
    // Save locale setting
    const std::string oldLocale=std::setlocale(LC_NUMERIC,nullptr);
    // Force '.' as the radix point. If you comment this out,
    // you'll get output similar to the OP's GUI mode sample
    std::setlocale(LC_NUMERIC,"C");
    try
    {
        lidar_model = this->configLoader.get<int>("lidar_model");
        msop_port = this->configLoader.get<int>("msop_port");
        difop_port = this->configLoader.get<int>("difop_port");
        dest_pc_ip_addr =this->configLoader.get<std::string>("dest_pc_ip_addr");

        //Extrinsic
        float rx, ry, rz, tx, ty, tz;
        rx = this->configLoader.get<double>("rx");
        ry = this->configLoader.get<double>("ry");
        rz = this->configLoader.get<double>("rz");
        tx = this->configLoader.get<double>("tx");
        ty = this->configLoader.get<double>("ty");
        tz = this->configLoader.get<double>("tz");
        this->robot_lidar = Eigen::Translation3f(Eigen::Vector3f(tx,ty,tz));
        this->robot_lidar.rotate(Eigen::AngleAxisf (rx,Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf (ry, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ()));
        std::cout<<"Extrinsec Matrix:"<<std::endl<<this->robot_lidar.matrix()<<std::endl;

        //boundin box colision / hitbox
        float center_box_x, center_box_y, center_box_z, size_box_x, size_box_y, size_box_z;
        center_box_x = this->configLoader.get<double>("center_box_x");
        center_box_y = this->configLoader.get<double>("center_box_y");
        center_box_z = this->configLoader.get<double>("center_box_z");
        size_box_x = this->configLoader.get<double>("size_box_x");
        size_box_y = this->configLoader.get<double>("size_box_y");
        size_box_z = this->configLoader.get<double>("size_box_z");

        simulator = this->configLoader.get<bool>("simulator");

        box_min.x() = center_box_x - size_box_x/2.0;//minx
        box_min.y() = center_box_y - size_box_y/2.0;//miny
        box_min.z() = center_box_z - size_box_z/2.0;//minz
        box_max.x() = center_box_x + size_box_x/2.0;//maxx
        box_max.y() = center_box_y + size_box_y/2.0;//maxy
        box_max.z() = center_box_z + size_box_z/2.0;//maxz

        floor_line = this->configLoader.get<double>("floor_line");
        top_line = this->configLoader.get<double>("top_line");

        std::cout<<"Hitbox min in millimetres:"<<std::endl<<this->box_min<<std::endl;
        std::cout<<"Hitbox max in millimetres:"<<std::endl<<this->box_max<<std::endl;
        std::cout<<"Floor line in millimetres:"<<std::endl<<this->floor_line<<std::endl;

        #if USE_OPEN3D
            // downsampling mm
            down_sampling = this->configLoader.get<double>("down_sampling");
        #else
            down_sampling = 0;
        #endif
        std::cout << "Downsampling in mm " << down_sampling << std::endl;


    }catch (const std::exception &e)
    {
        std::cout <<"Error reading the config \n" << e.what() << std::endl << std::flush;
        std::terminate();
        }

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
    if (not simulator)
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

        if (not driver.init(param))                         ///< Call the init function
        {
            std::cout << __FUNCTION__ << " Driver Initialize Error.... Aborting" << std::endl;
            std::terminate();
        }
        driver.start();
        std::cout << __FUNCTION__ << " Driver initiated OK" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(getPeriod("Compute")));
    ready_to_go.store(true);
}

void SpecificWorker::compute()
{
    std::pair<RoboCompLidar3D::TData, RoboCompLidar3D::TData> raw_lidar;
    int num_points = 0, num_erased_points = 0;

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
            raw_lidar.first.timestamp = raw_lidar_sim.timestamp;
            raw_lidar.second.timestamp = raw_lidar_sim.timestamp;
        }
        else // real lidar
        {
            std::shared_ptr <PointCloudMsg> msg = stuffed_cloud_queue.popWait();
            if (msg == nullptr)
                return;

            raw_lidar = processLidarData(*msg);

            auto now = std::chrono::system_clock::now();
            raw_lidar.first.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

            raw_lidar.second.timestamp = raw_lidar.first.timestamp;

        }
        raw_lidar.first.period = static_cast<float>(getPeriod("Compute")); // ms

        //Process lidar helios and add lidar data to doubleBuffer
        if (lidar_model==0)
        { //Helios
            std::pair<RoboCompLidar3D::TDataImage, RoboCompLidar3D::TDataImage> processed_real_lidar_array;
            processed_real_lidar_array.first = lidar2cam(raw_lidar.first);
            processed_real_lidar_array.second = lidar2cam(raw_lidar.second);
            buffer_array_data.put(std::move(processed_real_lidar_array));
        }

        num_points = raw_lidar.first.points.size();
        num_erased_points = raw_lidar.second.points.size();
        buffer_data.put(std::move(raw_lidar));
    }
    catch (const Ice::Exception &e)
        {std::cerr << __FUNCTION__ << " Error in Lidar Proxy\n" << e.what() << std::endl;}

    fps.print("Num points: " + std::to_string(num_points) + " | Num erased points: " +  std::to_string(num_erased_points)/*+ " Timestamp: " + std::to_string(raw_lidar.timestamp)*/, 3000);
}


void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
	//computeCODE
	//
	//if (SUCCESSFUL)
    //  emmit goToRestore()
}

//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
	//computeCODE
	//Restore emergency component

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
    if (msg != nullptr)
        return msg;
    else
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
    int period = getPeriod("Compute");
    if(abs(new_period - period) < 2)      // do it only if period changes
        return;
    if(new_period > period)
        setPeriod("Compute", period + 1);
    else
        setPeriod("Compute", period - 1);
}
std::pair<bool, Eigen::Vector3f> SpecificWorker::transform_filter_point(float x, float y, float z, int intensity)
{
    Eigen::Vector3f point(-y*1000, x*1000, z*1000); // convert to millimeters
    Eigen::Vector3f lidar_point = robot_lidar.linear() * point + robot_lidar.translation();  // Assumes no rotation

    const bool valid =
        isPointOutsideCube(lidar_point, box_min, box_max) &&
        lidar_point.z() > floor_line &&
        lidar_point.z() < top_line;

    return { valid, lidar_point };
}
std::pair<RoboCompLidar3D::TData, RoboCompLidar3D::TData> SpecificWorker::processLidarData(const auto &input_points)
{
    std::pair<RoboCompLidar3D::TData, RoboCompLidar3D::TData> raw_lidar; // <utils points, erased points>

    if (input_points.points.empty())
        return raw_lidar;

    if(this->down_sampling == 0)    // no downsampling
    {
        // filter out body points
       for (const auto& p : input_points.points)
        {
            auto [valid, pt] = transform_filter_point(p.x, p.y, p.z, p.intensity);

            const float x = pt.x(), y = pt.y(), z = pt.z();
            const float r = std::sqrt(x*x + y*y + z*z);
            const float distance2d = std::hypot(x, y);
            const float phi = std::atan2(x, y);
            const float theta = std::acos(z / r);

            RoboCompLidar3D::TPoint out{
                .x = x, .y = y, .z = z,
                .intensity = p.intensity,
                .phi = phi,
                .theta = theta,
                .r = r,
                .distance2d = distance2d
            };

            if (valid)
                raw_lidar.first.points.emplace_back(std::move(out));
            else
                raw_lidar.second.points.emplace_back(std::move(out));
        }

        std::ranges::sort(raw_lidar.first.points, {}, &RoboCompLidar3D::TPoint::phi);
        // std::ranges::sort(raw_lidar.second.points, {}, &RoboCompLidar3D::TPoint::phi);
        return raw_lidar;
    }


    #if USE_OPEN3D
        /// Downsampling
        // filter out body points
        auto pcd = std::make_shared<open3d::geometry::PointCloud>();
        for (const auto &p : input_points.points)
            if(auto transformed = transform_filter_point(p.x, p.y, p.z, p.intensity); transformed.has_value())
                pcd->points_.emplace_back(*transformed);

        // transform to Eigen::Vector3d for Open3D
        std::ranges::transform(raw_lidar.points, std::back_inserter(pcd->points_),
                            [](const auto &p) -> Eigen::Vector3d
                            { return {p.x, p.y, p.z};});
        shared_ptr<open3d::geometry::PointCloud> downsampled_pcd;
        try
        { downsampled_pcd = pcd->VoxelDownSample(this->down_sampling); }
        catch (const std::exception &e){std::cout << e.what() << std::endl;};

        // Transform back to RoboCompLidar3D::TData and fill. Check if downsampled is empty
        shared_ptr<open3d::geometry::PointCloud> pcd_points;
        if(not downsampled_pcd->IsEmpty())
            pcd_points = downsampled_pcd;
        else
            pcd_points = pcd;

        raw_lidar.points.reserve(pcd_points->points_.size());
        for (const auto &p: downsampled_pcd->points_)
        {
            auto distance2d = std::hypot(p.x(), p.y());
            auto r = p.norm();
            raw_lidar.points.emplace_back(RoboCompLidar3D::TPoint{.x=(float) p.x(), .y=(float) p.y(), .z=(float) p.z(),
                    .intensity=0, .phi=(float) std::atan2(p.x(), p.y()),
                    .theta=(float) std::acos(p.z() / r), .r=(float) r, .distance2d=(float) distance2d});
        }
        //qDebug() << "Original size: " << input_points.points.size() << " Downsampled size: " << raw_lidar.points.size();
        std::ranges::sort(raw_lidar.points, {}, &RoboCompLidar3D::TPoint::phi);
        return raw_lidar;
    #endif
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
// C++
RoboCompLidar3D::TDataImage SpecificWorker::lidar2cam(const RoboCompLidar3D::TData &lidar_data)
{
    // ---------------------------------------------------------
    // 1. EXTRINSIC CONFIGURATION (Position and Orientation)
    // ---------------------------------------------------------

    cv::Mat rvec(3, 1, CV_64F);
    cv::Mat tvec(3, 1, CV_64F);

    // Configure rotation and translation depending on environment
    if (simulator)
    {
        // Rotation for simulator: 90 degrees around X (Rodrigues vector)
        rvec.at<double>(0, 0) = M_PI_2;
        rvec.at<double>(1, 0) = 0.0;
        rvec.at<double>(2, 0) = 0.0;

        // Translation for simulator (in mm)
        tvec.at<double>(0, 0) = 0.0;
        tvec.at<double>(1, 0) = 1350.0;
        tvec.at<double>(2, 0) = 170.0;

        // Destination image resolution for simulator
        dst_width = 1920;
        dst_height = 960;
    }
    else // Real robot
    {
        // Rotation for real robot (Rodrigues vector)
        rvec.at<double>(0, 0) = M_PI_2;
        rvec.at<double>(1, 0) = 0.0;
        rvec.at<double>(2, 0) = 0.0;

        // Translation for real robot (in mm)
        tvec.at<double>(0, 0) = 0.0;
        tvec.at<double>(1, 0) = 1330.0;
        tvec.at<double>(2, 0) = 170.0;

        // Keep default destination resolution (can be adjusted if needed)
    }

    // ---------------------------------------------------------
    // 2. TRANSFORMATION MATRICES PREPARATION
    // ---------------------------------------------------------

    // Convert Rodrigues rotation vector to 3x3 rotation matrix
    cv::Mat R_mat;
    cv::Rodrigues(rvec, R_mat);

    // Copy rotation matrix to a float array for faster access in the loop
    float R[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R[i][j] = static_cast<float>(R_mat.at<double>(i, j));

    // Copy translation vector to float array
    float T[3];
    T[0] = static_cast<float>(tvec.at<double>(0, 0));
    T[1] = static_cast<float>(tvec.at<double>(1, 0));
    T[2] = static_cast<float>(tvec.at<double>(2, 0));

    // ---------------------------------------------------------
    // 3. POINTS PROCESSING AND EQUIRECTANGULAR PROJECTION
    // ---------------------------------------------------------

    RoboCompLidar3D::TDataImage data_image;
    data_image.timestamp = lidar_data.timestamp;
    size_t num_points = lidar_data.points.size();

    // Reserve vector capacity to avoid reallocations
    data_image.XArray.reserve(num_points);
    data_image.YArray.reserve(num_points);
    data_image.ZArray.reserve(num_points);
    data_image.XPixel.reserve(num_points);
    data_image.YPixel.reserve(num_points);

    const float inv_PI = 1.0f / M_PI;
    const float inv_2PI = 1.0f / (2.0f * M_PI);

    for (const auto &p : lidar_data.points)
    {
        // A. Extrinsic transform: P_cam = R * P_lidar + T
        // Assumes p.x, p.y, p.z are in the same units as T (mm).
        float x_cam = R[0][0] * p.x + R[0][1] * p.y + R[0][2] * p.z + T[0];
        float y_cam = R[1][0] * p.x + R[1][1] * p.y + R[1][2] * p.z + T[1];
        float z_cam = R[2][0] * p.x + R[2][1] * p.y + R[2][2] * p.z + T[2];

        // Compute planar distance and full 3D distance in camera coords
        float rho = std::hypot(x_cam, y_cam);
        float r_dist = std::sqrt(x_cam * x_cam + y_cam * y_cam + z_cam * z_cam);

        // Skip very close noise points
        if (r_dist < 10.0f)
            continue;

        // NOTE: After the applied rotation, typical camera axes may be:
        // X = right, Y = down, Z = forward. Adjust atan2 usage if different.
        // Here we compute yaw (longitude) and pitch (latitude) for equirectangular mapping.

        // Yaw: angle around vertical axis — use X and Z (X to the right, Z forward)
        float yaw = std::atan2(x_cam, z_cam);

        // Pitch: vertical angle — use negative Y if camera Y is downwards
        float pitch = std::atan2(-y_cam, std::hypot(x_cam, z_cam));

        // Normalize to [0,1] UV coordinates for equirectangular image
        float u = (yaw * inv_2PI) + 0.5f;
        float v = 0.5f - (pitch * inv_PI);

        // Convert UV to pixel coordinates
        int px = static_cast<int>(u * dst_width);
        int py = static_cast<int>(v * dst_height);

        // Clamp pixel indices to image bounds
        px = std::max(0, std::min(px, dst_width - 1));
        py = std::max(0, std::min(py, dst_height - 1));

        // Store original LiDAR coordinates (or change to transformed coords if desired)
        data_image.XArray.push_back(p.x);
        data_image.YArray.push_back(p.y);
        data_image.ZArray.push_back(p.z);
        data_image.XPixel.push_back(px);
        data_image.YPixel.push_back(py);

    }

    return data_image;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////// Interfaces                                                            //////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RoboCompLidar3D::TColorCloudData SpecificWorker::Lidar3D_getColorCloudData()
{
    RoboCompLidar3D::TColorCloudData cloud;

    //merge all points
    auto [data_valid, data_invalid] = buffer_data.get_idemp();
    const auto total_size = data_valid.points.size() + data_invalid.points.size();
    std::vector<RoboCompLidar3D::TPoint> points;
    points.reserve(total_size);

    points.insert(points.end(),
                         std::make_move_iterator(data_valid.points.begin()),
                         std::make_move_iterator(data_valid.points.end()));

    points.insert(points.end(),
                         std::make_move_iterator(data_invalid.points.begin()),
                         std::make_move_iterator(data_invalid.points.end()));


    const size_t N = points.size();

    cloud.X.resize(N);
    cloud.Y.resize(N);
    cloud.Z.resize(N);
    cloud.R.resize(N);
    cloud.G.resize(N);
    cloud.B.resize(N);

    constexpr float maxShort = 32767.0f;

    #pragma omp parallel for
    for (size_t i = 0; i < points.size(); ++i)
    {
        const auto& p = points[i];
        cloud.X[i] = static_cast<int16_t>(std::clamp(p.x, -maxShort, maxShort));
        cloud.Y[i] = static_cast<int16_t>(std::clamp(p.y, -maxShort, maxShort));
        cloud.Z[i] = static_cast<int16_t>(std::clamp(p.z, -maxShort, maxShort));
        cloud.R[i] = cloud.G[i] = cloud.B[i] = 255;
    }

	return cloud;
}

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

    RoboCompLidar3D::TData buffer = buffer_data.get_idemp().first;

    // Check for nominal conditions
    if(len >= 2.0*M_PI  and decimationDegreeFactor == 1)
        return buffer;

    RoboCompLidar3D::TPoints filtered_points;

    //Get all LiDAR
    if (len >= 2.0*M_PI)
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
    RoboCompLidar3D::TData buffer = buffer_data.get_idemp().first;

    //Sort distance and cut
    std::ranges::sort(buffer.points, {}, &RoboCompLidar3D::TPoint::distance2d);
    RoboCompLidar3D::TPoints filtered_points(std::make_move_iterator(buffer.points.begin()), std::make_move_iterator(
            std::find_if(buffer.points.begin(), buffer.points.end(),
                         [_distance=distance](const RoboCompLidar3D::TPoint& point)
                         {return _distance < point.distance2d;})));
    //ReSort by phi
    std::ranges::sort(filtered_points, {}, &RoboCompLidar3D::TPoint::phi);

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
RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataProyectedInImage(std::string name)
{
    //LiDAR not started
    if (not ready_to_go)
        return {};

    RoboCompLidar3D::TData buffer;
    RoboCompLidar3D::TPoints processed_points;

    //Get LiDAR data
    return buffer_data.get_idemp().first;
}

RoboCompLidar3D::TDataCategory SpecificWorker::Lidar3D_getLidarDataByCategory(RoboCompLidar3D::TCategories categories, Ice::Long timestamp)
{
	RoboCompLidar3D::TDataCategory ret{};
	//implementCODE

	return ret;
}
RoboCompLidar3D::TDataImage SpecificWorker::Lidar3D_getLidarDataArrayProyectedInImage(std::string name)
{

    //LiDAR not started
    if(not ready_to_go)
        return {};

    if (name == "unfiltered"){

        auto [data_valid, data_invalid] = buffer_array_data.get_idemp();
        const auto total_size = data_valid.XArray.size() + data_invalid.XArray.size();
        RoboCompLidar3D::TDataImage dataImage;
        dataImage.timestamp = std::max(data_valid.timestamp, data_invalid.timestamp);

        dataImage.XArray.reserve(total_size);
        dataImage.YArray.reserve(total_size);
        dataImage.ZArray.reserve(total_size);
        dataImage.XPixel.reserve(total_size);
        dataImage.YPixel.reserve(total_size);

        dataImage.XArray.insert(dataImage.XArray.end(),
            std::make_move_iterator(data_valid.XArray.begin()),
            std::make_move_iterator(data_valid.XArray.end()));
        dataImage.XArray.insert(dataImage.XArray.end(),
            std::make_move_iterator(data_invalid.XArray.begin()),
            std::make_move_iterator(data_invalid.XArray.end()));

        dataImage.YArray.insert(dataImage.YArray.end(),
            std::make_move_iterator(data_valid.YArray.begin()),
            std::make_move_iterator(data_valid.YArray.end()));
        dataImage.YArray.insert(dataImage.YArray.end(),
            std::make_move_iterator(data_invalid.YArray.begin()),
            std::make_move_iterator(data_invalid.YArray.end()));

        dataImage.ZArray.insert(dataImage.ZArray.end(),
            std::make_move_iterator(data_valid.ZArray.begin()),
            std::make_move_iterator(data_valid.ZArray.end()));
        dataImage.ZArray.insert(dataImage.ZArray.end(),
            std::make_move_iterator(data_invalid.ZArray.begin()),
            std::make_move_iterator(data_invalid.ZArray.end()));

        dataImage.XPixel.insert(dataImage.XPixel.end(),
            std::make_move_iterator(data_valid.XPixel.begin()),
            std::make_move_iterator(data_valid.XPixel.end()));
        dataImage.XPixel.insert(dataImage.XPixel.end(),
            std::make_move_iterator(data_invalid.XPixel.begin()),
            std::make_move_iterator(data_invalid.XPixel.end()));

        dataImage.YPixel.insert(dataImage.YPixel.end(),
            std::make_move_iterator(data_valid.YPixel.begin()),
            std::make_move_iterator(data_valid.YPixel.end()));
        dataImage.YPixel.insert(dataImage.YPixel.end(),
            std::make_move_iterator(data_invalid.YPixel.begin()),
            std::make_move_iterator(data_invalid.YPixel.end()));
        return dataImage;
    }
    else{
        return buffer_array_data.get_idemp().first;
    }
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// RoboCompLidar3D::TColorCloudData this->lidar3d_proxy->getColorCloudData()
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarData(string name, float start, float len, int decimationDegreeFactor)
// RoboCompLidar3D::TDataImage this->lidar3d_proxy->getLidarDataArrayProyectedInImage(string name)
// RoboCompLidar3D::TDataCategory this->lidar3d_proxy->getLidarDataByCategory(TCategories categories, long timestamp)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataProyectedInImage(string name)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataWithThreshold2d(string name, float distance, int decimationDegreeFactor)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData
