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
#include "dds_publisher.h"
#include "mesh_filter.h"
#include <cppitertools/enumerate.hpp>
#include <algorithm>
#include <optional>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

robosense::lidar::SyncQueue <std::shared_ptr<PointCloudMsg>> free_cloud_queue;
robosense::lidar::SyncQueue <std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

namespace
{
// Build the lidar->robot extrinsic (Translation * RotXYZ), matching the legacy rx..tz path.
Eigen::Affine3f build_extrinsic(float rx, float ry, float rz, float tx, float ty, float tz)
{
    Eigen::Affine3f a{Eigen::Translation3f(tx, ty, tz)};
    a.rotate(Eigen::AngleAxisf(rx, Eigen::Vector3f::UnitX())
           * Eigen::AngleAxisf(ry, Eigen::Vector3f::UnitY())
           * Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ()));
    return a;
}

// Read a sensor's static mount (lidar -> robot) from the DSR shadow.json — the single
// source of geometry. Finds the node named `node_name`, then the incoming RT edge, and
// reads linkAttribute.rt_rotation_euler_xyz.value (rad) + rt_translation.value (METRES,
// converted to mm here). Returns nullopt on any problem (missing file/node/edge/fields)
// so the caller can roll back to the config extrinsics.
std::optional<Eigen::Affine3f> load_mount_from_shadow(const std::string& path, const std::string& node_name)
{
    QFile f(QString::fromStdString(path));
    if (not f.open(QIODevice::ReadOnly))
        return std::nullopt;

    QJsonParseError err;
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
    if (err.error != QJsonParseError::NoError or not doc.isObject())
        return std::nullopt;

    const QJsonObject symbols = doc.object().value("DSRModel").toObject().value("symbols").toObject();
    if (symbols.isEmpty())
        return std::nullopt;

    // node name -> id
    const QString want = QString::fromStdString(node_name);
    QString target_id;
    for (auto it = symbols.begin(); it != symbols.end(); ++it)
        if (it.value().toObject().value("name").toString() == want) { target_id = it.key(); break; }
    if (target_id.isEmpty())
        return std::nullopt;

    // find the RT edge whose dst is that node
    for (auto it = symbols.begin(); it != symbols.end(); ++it)
        for (const auto& lv : it.value().toObject().value("links").toArray())
        {
            const QJsonObject lk = lv.toObject();
            if (lk.value("label").toString() != "RT" or lk.value("dst").toString() != target_id)
                continue;
            const QJsonObject la = lk.value("linkAttribute").toObject();
            const QJsonArray rot = la.value("rt_rotation_euler_xyz").toObject().value("value").toArray();
            const QJsonArray tr  = la.value("rt_translation").toObject().value("value").toArray();
            if (rot.size() != 3 or tr.size() != 3)
                return std::nullopt;
            return build_extrinsic(
                static_cast<float>(rot[0].toDouble()), static_cast<float>(rot[1].toDouble()), static_cast<float>(rot[2].toDouble()),
                static_cast<float>(tr[0].toDouble() * 1000.0), static_cast<float>(tr[1].toDouble() * 1000.0), static_cast<float>(tr[2].toDouble() * 1000.0));
        }
    return std::nullopt;
}
}  // namespace

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
    try
    {
        lidar_model = this->configLoader.get<int>("lidar_model");
        msop_port = this->configLoader.get<int>("msop_port");
        difop_port = this->configLoader.get<int>("difop_port");
        dest_pc_ip_addr =this->configLoader.get<std::string>("dest_pc_ip_addr");

        // Extrinsic (lidar -> robot). Single source of truth: read the STATIC mount
        // from a JSON file (mount_file) keyed by sensor name (mount_key, defaults to
        // helios/bpearl from lidar_model). If there is no file / it can't be read,
        // roll back to the config rx..tz extrinsics and say so on start.
        const float rx = this->configLoader.get<double>("rx");
        const float ry = this->configLoader.get<double>("ry");
        const float rz = this->configLoader.get<double>("rz");
        const float tx = this->configLoader.get<double>("tx");
        const float ty = this->configLoader.get<double>("ty");
        const float tz = this->configLoader.get<double>("tz");

        std::string mount_file, mount_key;
        try { mount_file = this->configLoader.get<std::string>("mount_file"); } catch(...) {}
        try { mount_key  = this->configLoader.get<std::string>("mount_key");  } catch(...) {}
        if (mount_key.empty())
            mount_key = (lidar_model == 1) ? "bpearl" : "helios";

        bool from_file = false;
        if (not mount_file.empty())
        {
            if (auto m = load_mount_from_shadow(mount_file, mount_key); m.has_value())
            {
                this->robot_lidar = *m;
                from_file = true;
                std::cout << "[Mount] loaded static mount '" << mount_key << "' from " << mount_file << std::endl;
            }
            else
                std::cerr << "[Mount] could not read '" << mount_key << "' from '" << mount_file
                          << "' — rolling back to config extrinsics (rx..tz)" << std::endl;
        }
        else
            std::cout << "[Mount] no mount_file configured — using config extrinsics (rx..tz)" << std::endl;

        if (not from_file)
            this->robot_lidar = build_extrinsic(rx, ry, rz, tx, ty, tz);

        std::cout << "Extrinsic Matrix:" << std::endl << this->robot_lidar.matrix() << std::endl;

        simulator = this->configLoader.get<bool>("simulator");


        // ---------------------------------------------------------
        // EXTRINSIC CONFIGURATION (Position and Orientation)
        // ---------------------------------------------------------
        rvec.create(3, 1, CV_64F);  
        tvec.create(3, 1, CV_64F);

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

            // Destination image resolution for real robot 
            dst_width = 1920;
            dst_height = 920;
        }

    }catch (const std::exception &e)
    {
        std::cout <<"Error reading the config \n" << e.what() << std::endl << std::flush;
        std::terminate();
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

    // ---- Optional robot-body self-filter (Embree, static Shadow mesh) ----
    // Removes points hitting the robot body; folded in from lidar3DFilter to cut its
    // decoupled-component latency. Applied once in compute() before buffer + DDS.
    try { mesh_filter_enabled = this->configLoader.get<bool>("MeshFilter.enabled"); }
    catch(...) { mesh_filter_enabled = false; }
    if (mesh_filter_enabled)
    {
        MeshFilter::Config mcfg;
        mcfg.mount = this->robot_lidar;   // device -> robot mount (from shadow.json / config)
        try { mcfg.robot_name = this->configLoader.get<std::string>("Robot.name"); } catch(...) {}
        try { mcfg.mesh_dir   = this->configLoader.get<std::string>("MeshFilter.mesh_dir"); } catch(...) {}
        try { mcfg.floor_z    = static_cast<float>(this->configLoader.get<double>("Floor.z")); } catch(...) {}
        try { mcfg.top_z      = static_cast<float>(this->configLoader.get<double>("Top.z")); } catch(...) {}
        try { mcfg.dilate     = static_cast<float>(this->configLoader.get<double>("Dilate")); } catch(...) {}
        // Robot-frame footprint disc (drops base/wheels/arm returns the STL doesn't model).
        try { mcfg.footprint_radius = static_cast<float>(this->configLoader.get<double>("Footprint.radius")); } catch(...) {}
        try { mcfg.footprint_z_min  = static_cast<float>(this->configLoader.get<double>("Footprint.z_min")); } catch(...) {}
        try { mcfg.footprint_z_max  = static_cast<float>(this->configLoader.get<double>("Footprint.z_max")); } catch(...) {}
        // Wider near-floor skirt (drops near-floor self / ground-reflection returns).
        try { mcfg.skirt_radius = static_cast<float>(this->configLoader.get<double>("Skirt.radius")); } catch(...) {}
        try { mcfg.skirt_z_min  = static_cast<float>(this->configLoader.get<double>("Skirt.z_min")); } catch(...) {}
        try { mcfg.skirt_z_max  = static_cast<float>(this->configLoader.get<double>("Skirt.z_max")); } catch(...) {}

        mesh_filter = std::make_unique<MeshFilter>();
        if (not mesh_filter->init(mcfg))
        {
            std::cerr << "MeshFilter failed to initialize; disabling self-filtering." << std::endl;
            mesh_filter.reset();
            mesh_filter_enabled = false;
        }
    }

    // ---- Optional DDS lidar media plane (LidarFrame into the CORTEX stack) ----
    // Gated by PublishDDS. Metadata is advertised into DSR by robot_concept, not here.
    try { publish_dds = this->configLoader.get<bool>("PublishDDS"); }
    catch(...) { publish_dds = false; }
    if (publish_dds)
    {
        LidarDDSPublisher::Config dcfg;   // defaults: domain 7, rc/lidar3d/points
        try { dcfg.domain_id = static_cast<std::uint32_t>(this->configLoader.get<int>("DDS.Domain")); } catch(...) {}
        try { dcfg.topic = this->configLoader.get<std::string>("DDS.Topic"); } catch(...) {}
        try { dcfg.history_depth = this->configLoader.get<int>("DDS.HistoryDepth"); } catch(...) {}
        try { dcfg.shared_memory_only = this->configLoader.get<bool>("DDS.SharedMemoryOnly"); } catch(...) {}
        try { dcfg.data_sharing = this->configLoader.get<bool>("DDS.DataSharing"); } catch(...) {}

        dds_publisher = std::make_unique<LidarDDSPublisher>();
        if (not dds_publisher->init(dcfg))
        {
            std::cerr << "DDS lidar media plane failed to initialize; disabling DDS publishing." << std::endl;
            dds_publisher.reset();
            publish_dds = false;
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(getPeriod("Compute")));
    ready_to_go.store(true);
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
        else // real lidar
        {
            std::shared_ptr <PointCloudMsg> msg = stuffed_cloud_queue.popWait();
            if (msg == nullptr)
                return;

            raw_lidar = processLidarData(*msg);

            auto now = std::chrono::system_clock::now();
            raw_lidar.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

        }
        raw_lidar.period = static_cast<float>(getPeriod("Compute")); // ms

        // Self-filter: drop points hitting the robot body mesh (once, upstream of every
        // output — the image projection, the Ice buffer and the DDS plane all get it).
        if (mesh_filter_enabled && mesh_filter)
            mesh_filter->filter(raw_lidar);

        //Process lidar helios and add lidar data to doubleBuffer
        if (lidar_model==0)
        { //Helios
            RoboCompLidar3D::TDataImage processed_real_lidar_array;
            processed_real_lidar_array = lidar2cam(raw_lidar);
            buffer_array_data.put(std::move(processed_real_lidar_array));
        }

        num_points = raw_lidar.points.size();

        // Publish the scan on the zero-copy DDS media plane (interleaved xyz, mm -> m).
        if (publish_dds && dds_publisher && not raw_lidar.points.empty())
        {
            const std::size_t n = raw_lidar.points.size();
            lidar_xyz.resize(n * 3);
            const auto &pts = raw_lidar.points;
            #pragma omp parallel for schedule(static)
            for (std::size_t i = 0; i < n; ++i)
            {
                lidar_xyz[3 * i + 0] = pts[i].x * 0.001f;   // mm -> m
                lidar_xyz[3 * i + 1] = pts[i].y * 0.001f;
                lidar_xyz[3 * i + 2] = pts[i].z * 0.001f;
            }
            dds_publisher->publish(static_cast<std::uint64_t>(raw_lidar.timestamp),
                                   lidar_xyz.data(), static_cast<std::uint32_t>(n));
        }

        buffer_data.put(std::move(raw_lidar));
    }
    catch (const Ice::Exception &e)
        {std::cerr << __FUNCTION__ << " Error in Lidar Proxy\n" << e.what() << std::endl;}

    fps.print("Num points: " + std::to_string(num_points)+ " Timestamp: " + std::to_string(raw_lidar.timestamp), 3000);
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

RoboCompLidar3D::TData SpecificWorker::processLidarData(const auto &input_points)
{
    RoboCompLidar3D::TData raw_lidar;
    const auto &in = input_points.points;
    const std::size_t n = in.size();
    if (n == 0)
        return raw_lidar;

    // DEVICE frame: the mount is NOT applied here — it lives once in the DSR RT edge
    // (shadow.json, robot->lidar) that consumers apply. We only put the sensor axes into
    // the canonical convention (swap) and metres->mm, then compute angles in this frame.
    // The mesh filter re-applies the mount (read from shadow.json) for its robot-frame test.
    raw_lidar.points.resize(n);
    #pragma omp parallel for schedule(static)
    for (std::size_t i = 0; i < n; ++i)
    {
        const auto &p = in[i];
        const float x = -p.y * 1000.0f;   // device frame, mm
        const float y =  p.x * 1000.0f;
        const float z =  p.z * 1000.0f;
        const float r = std::sqrt(x*x + y*y + z*z);
        raw_lidar.points[i] = RoboCompLidar3D::TPoint{
            .x = x, .y = y, .z = z,
            .intensity = p.intensity,
            .phi = std::atan2(x, y),
            .theta = (r > 0.f) ? std::acos(std::clamp(z / r, -1.0f, 1.0f)) : 0.0f,
            .r = r,
            .distance2d = std::hypot(x, y)
        };
    }

    // Ordered by phi as the interface expects.
    std::ranges::sort(raw_lidar.points, {}, &RoboCompLidar3D::TPoint::phi);
    return raw_lidar;
}

// C++
RoboCompLidar3D::TDataImage SpecificWorker::lidar2cam(const RoboCompLidar3D::TData &lidar_data)
{
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
    const size_t num_points = lidar_data.points.size();

    const float inv_PI = 1.0f / M_PI;
    const float inv_2PI = 1.0f / (2.0f * M_PI);

    // Parallel per-point equirectangular projection (independent); compact sequentially.
    const auto &pts = lidar_data.points;
    std::vector<int> px(num_points), py(num_points);
    std::vector<uint8_t> keep(num_points);
    #pragma omp parallel for schedule(static)
    for (size_t i = 0; i < num_points; ++i)
    {
        const auto &p = pts[i];
        // Extrinsic transform: P_cam = R * P_lidar + T (same units as T, mm).
        const float x_cam = R[0][0]*p.x + R[0][1]*p.y + R[0][2]*p.z + T[0];
        const float y_cam = R[1][0]*p.x + R[1][1]*p.y + R[1][2]*p.z + T[1];
        const float z_cam = R[2][0]*p.x + R[2][1]*p.y + R[2][2]*p.z + T[2];

        // Skip very close noise points.
        if (x_cam*x_cam + y_cam*y_cam + z_cam*z_cam < 100.0f) { keep[i] = 0; continue; }

        // Yaw (longitude) / pitch (latitude) → [0,1] UV → pixel, clamped to image bounds.
        const float yaw   = std::atan2(x_cam, z_cam);
        const float pitch = std::atan2(-y_cam, std::hypot(x_cam, z_cam));
        const float u = yaw * inv_2PI + 0.5f;
        const float v = 0.5f - pitch * inv_PI;
        px[i] = std::clamp(static_cast<int>(u * dst_width),  0, dst_width  - 1);
        py[i] = std::clamp(static_cast<int>(v * dst_height), 0, dst_height - 1);
        keep[i] = 1;
    }

    data_image.XArray.reserve(num_points);
    data_image.YArray.reserve(num_points);
    data_image.ZArray.reserve(num_points);
    data_image.XPixel.reserve(num_points);
    data_image.YPixel.reserve(num_points);
    for (size_t i = 0; i < num_points; ++i)
        if (keep[i])
        {
            const auto &p = pts[i];
            data_image.XArray.push_back(p.x);
            data_image.YArray.push_back(p.y);
            data_image.ZArray.push_back(p.z);
            data_image.XPixel.push_back(px[i]);
            data_image.YPixel.push_back(py[i]);
        }

    return data_image;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////// Interfaces                                                            //////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::string SpecificWorker::MediaPlaneDDS_getMediaDescriptor()
{
    // Report the live DDS lidar media-plane descriptor (JSON) so the robot_concept agent
    // can relay it into DSR. Empty when DDS publishing is disabled/failed, which tells
    // the consumer to fall back to bridging the scan itself.
    if (dds_publisher)
        return dds_publisher->descriptor_json();
    return {};
}

RoboCompLidar3D::TColorCloudData SpecificWorker::Lidar3D_getColorCloudData()
{
    RoboCompLidar3D::TColorCloudData cloud;

    //merge all points
    RoboCompLidar3D::TData buffer = buffer_data.get_idemp();
    const size_t N = buffer.points.size();

    cloud.X.resize(N);
    cloud.Y.resize(N);
    cloud.Z.resize(N);  
    cloud.R.resize(N);
    cloud.G.resize(N);
    cloud.B.resize(N);

    constexpr float maxShort = 32767.0f;

    #pragma omp parallel for
    for (size_t i = 0; i < N; ++i)
    {
        const auto& p = buffer.points[i];
        cloud.X[i] = static_cast<int16_t>(std::clamp(p.x, -maxShort, maxShort));
        cloud.Y[i] = static_cast<int16_t>(std::clamp(p.y, -maxShort, maxShort));
        cloud.Z[i] = static_cast<int16_t>(std::clamp(p.z, -maxShort, maxShort));
        cloud.R[i] = cloud.G[i] = cloud.B[i] = 255;
    }

    cloud.timestamp = buffer.timestamp;
    cloud.numberPoints = N;
    cloud.compressed = false;

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

    RoboCompLidar3D::TData buffer = buffer_data.get_idemp();

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
    RoboCompLidar3D::TData buffer = buffer_data.get_idemp();

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
    return buffer_data.get_idemp();
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

    return buffer_array_data.get_idemp();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////

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
