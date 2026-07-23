/*
 *    Copyright (C) 2026 by YOUR NAME HERE
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
#include <thread>
#include <chrono>
#include <cstring>
#include <algorithm>
#include "Eigen/Dense"

namespace
{
// Resize the six parallel color-cloud arrays in one place (build fills to the full
// frame size; store trims to the valid point count).
inline void resize_color_cloud(RoboCompLidar3D::TColorCloudData &c, std::size_t n)
{
    c.X.resize(n); c.Y.resize(n); c.Z.resize(n);
    c.R.resize(n); c.G.resize(n); c.B.resize(n);
}
}  // namespace


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
    running = false;
    zed.close();
}

void SpecificWorker::initialize()
{   
    qInfo() << "Initializing SpecificWorker";

    simulated = configLoader.get<bool>("Config.Simulated");
    display = configLoader.get<bool>("Config.Display");
    try {
        compute_xyz = configLoader.get<bool>("Config.ComputeXYZ");
    }
    catch (const std::runtime_error& e) {
        std::cerr << "ComputeXYZ does not found, using default true. " << e.what() << std::endl;
        compute_xyz = true;
    }
    try {
        publish_rgbd = configLoader.get<bool>("Config.Publish");
    }
    catch (const std::runtime_error& e) {
        std::cerr << "Publish does not found, using default true. " << e.what() << std::endl;
        publish_rgbd = true;
    }
    try {
        compute_color_cloud = configLoader.get<bool>("Config.ComputeColorCloud");
    }
    catch (const std::runtime_error& e) {
        std::cerr << "ComputeColorCloud does not found, using default false. " << e.what() << std::endl;
        compute_color_cloud = false;
    }
    try {
        camera_fps = configLoader.get<int>("Config.FPS");
    }
    catch (const std::runtime_error& e) {
        std::cerr << "FPS not found, using default 30. " << e.what() << std::endl;
        camera_fps = 30;
    }
    // Config.FPS is the single rate knob. Seed the compute-loop period from it: in sim
    // this is the rate the regulator then fine-tunes; for the real camera it just keeps
    // the timer from throttling below the hardware fps (grab() paces the loop). Only
    // call setPeriod when it actually differs, to avoid a redundant "Period ... changed"
    // log. The real ZED also runs at this fps (init_parameters.camera_fps below).
    if (camera_fps > 0 && getPeriod("Compute") != 1000 / camera_fps)
        setPeriod("Compute", 1000 / camera_fps);

    // ---- Optional DDS media plane (zero-copy RGB+depth into the CORTEX stack) ----
    // Gated: only brought up when Config.PublishDDS is true. Domain/topics are
    // configurable; defaults target the CORTEX media domain (7) and rc/zed/* topics.
    try {
        publish_dds = configLoader.get<bool>("Config.PublishDDS");
    }
    catch (const std::runtime_error& e) {
        std::cerr << "PublishDDS not found, using default false. " << e.what() << std::endl;
        publish_dds = false;
    }
    if (publish_dds)
    {
        ZedDDSPublisher::Config dds_cfg;   // defaults: domain 7, rc/zed/rgb, rc/zed/depth
        try { dds_cfg.domain_id = static_cast<std::uint32_t>(configLoader.get<int>("DDS.Domain")); }
        catch (const std::runtime_error&) { /* keep default */ }
        try { dds_cfg.rgb_topic = configLoader.get<std::string>("DDS.RGBTopic"); }
        catch (const std::runtime_error&) { /* keep default */ }
        try { dds_cfg.depth_topic = configLoader.get<std::string>("DDS.DepthTopic"); }
        catch (const std::runtime_error&) { /* keep default */ }
        // QoS (optional; must match the consumers reading the relayed descriptor)
        try { dds_cfg.history_depth = configLoader.get<int>("DDS.HistoryDepth"); }
        catch (const std::runtime_error&) { /* keep default */ }
        try { dds_cfg.shared_memory_only = configLoader.get<bool>("DDS.SharedMemoryOnly"); }
        catch (const std::runtime_error&) { /* keep default */ }
        try { dds_cfg.data_sharing = configLoader.get<bool>("DDS.DataSharing"); }
        catch (const std::runtime_error&) { /* keep default */ }
        // RGB byte order label: "rgb" for both the real ZED (BGRA->RGB converted) and the Webots
        // CameraRGBDSimple proxy (RGB-ordered); "bgr" only for a source emitting genuine BGR.
        try {
            const std::string fmt = configLoader.get<std::string>("DDS.RGBFormat");
            dds_cfg.rgb_is_bgr = (fmt == "bgr" || fmt == "BGR");
        }
        catch (const std::runtime_error&) { /* keep default (rgb) */ }

        dds_publisher = std::make_unique<ZedDDSPublisher>();
        if (!dds_publisher->init(dds_cfg))
        {
            std::cerr << "DDS media plane failed to initialize; disabling DDS publishing." << std::endl;
            dds_publisher.reset();
            publish_dds = false;
        }
    }

    std::vector<double> extrinsicVector;
    try {
        extrinsicVector = configLoader.get<std::vector<double>>("Config.Extrinsic");
        }
    catch (const std::runtime_error& e) {
        std::cerr << "Extrinsic does not found, using default [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" << e.what()<< std::endl;
        extrinsicVector = {0,0,0,0,0,0};
    }

    // Asegurarse de que tenga al menos 6 elementos
    if (extrinsicVector.size() < 6) {
        throw std::runtime_error("Config.Extrinsic must be 6 values (x, y, z, rotX, rotY, rotZ)");
    }

    Eigen::Vector3f translation(
        static_cast<float>(extrinsicVector[0]),
        static_cast<float>(extrinsicVector[1]),
        static_cast<float>(extrinsicVector[2])
    );

    Eigen::AngleAxisf rotX(static_cast<float>(extrinsicVector[3]), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rotY(static_cast<float>(extrinsicVector[4]), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rotZ(static_cast<float>(extrinsicVector[5]), Eigen::Vector3f::UnitZ());
    Eigen::Affine3f extrinsic = Eigen::Translation3f(translation) * rotX * rotY * rotZ;

    this->extrinsic = extrinsic;
    std::cout<<"Extrinsic Matrix:"<<std::endl<<this->extrinsic.matrix()<<std::endl;



    if (!simulated)
    {
        // INIT PARAMETERS
        init_parameters.camera_resolution = sl::RESOLUTION::HD720; // Use HD720 opr HD1200 video mode, depending on camera type.
        init_parameters.camera_fps = camera_fps; // Config.FPS (ZED supports 15/30/60/100 per resolution)
        #ifdef POSE
            init_parameters.depth_mode = sl::DEPTH_MODE::NEURAL_PLUS; // Use ULTRA depth mode
        #else
            init_parameters.depth_mode = sl::DEPTH_MODE::NEURAL_PLUS; // Use ULTRA depth mode
        #endif
        init_parameters.coordinate_units = sl::UNIT::MILLIMETER; // Use millimeter units (for depth measurements)
        init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP; // Use a right-handed Y-up coordinate system
        init_parameters.enable_image_enhancement = true; // Mejora visual en condiciones complejas
        //init_parameters.async_grab_camera_recovery = false; // Evita bloqueos si hay pérdida temporal de conexión
        //init_parameters.camera_disable_self_calib = false; // Habilitar autocalibración mejora tracking en largo plazo

    
        // Abrir cámara
        returned_state = zed.open(init_parameters);
        if (returned_state != sl::ERROR_CODE::SUCCESS) {
            std::cout << "Error " << returned_state << ", exit program." << std::endl;
            exit(0);
        }

        // TRACKING PARAMETERS
        #ifndef POSE
            //init_parameters.camera_fps = 60; // 30 FPS balancea precisión y estabilidad

            tracking_parameters.enable_imu_fusion = true; // Combina visual e IMU
            tracking_parameters.enable_pose_smoothing = true; // Estabiliza la pose (algo más lento)
            tracking_parameters.set_as_static = false; // Asegura que use movimiento real
            tracking_parameters.mode = sl::POSITIONAL_TRACKING_MODE::GEN_2; // Último modo de seguimiento (mejor IMU)
            tracking_parameters.enable_area_memory = true;

        #endif

        auto err = zed.enablePositionalTracking(tracking_parameters);
        if (err != sl::ERROR_CODE::SUCCESS)
            exit(-1);

        sl::Transform reset_transform;
        zed.resetPositionalTracking(reset_transform);

        // Get the distance between the center of the camera and the left eye

        sl::CameraInformation cam_info = zed.getCameraInformation();

        cam_params = cam_info.camera_configuration.calibration_parameters.left_cam;
        translation_left_to_center = cam_info.camera_configuration.calibration_parameters.stereo_transform;
        // Lanzar hilo
        running = true;

    //IMU, barometer and magnetometer
    // sensor_thread = std::thread(&SpecificWorker::sensorsLoop, this);
    }
    qInfo() << "Initialization completed for SpecificWorker";
}

// Clean orchestrator: run the active source branch, then report FPS.
//  - Simulated: getAll() returns immediately, so the state-machine period sets the
//    rate -> the closed-loop regulator drives it toward Config.FPS.
//  - Real ZED: zed.grab() BLOCKS until the next hardware frame (init_parameters.
//    camera_fps), so the camera itself paces the loop. Adjusting the state-machine
//    period cannot change that rate, so we do NOT regulate here (it would just chase
//    grab's blocking time and spam setPeriod). The period is seeded in initialize()
//    to 1000/FPS so the timer never throttles below the hardware rate.
void SpecificWorker::compute()
{
    if (simulated)
    {
        regulate_period();
        step_simulated();
    }
    else
        step_real();

    fps.print("ZED compute");
}

// Closed-loop rate control: nudge the Compute state-machine period so the MEASURED
// loop rate converges on Config.FPS, compensating for the per-cycle processing/timer
// overhead a fixed period ignores. Proportional control on a smoothed period; called
// once per cycle (single-threaded), so a function-local clock is safe.
void SpecificWorker::regulate_period()
{
    if (camera_fps <= 0)
        return;
    const double target = 1000.0 / camera_fps;     // desired ms/frame

    static auto   last  = std::chrono::steady_clock::time_point{};
    static double ema   = target;                  // smoothed measured loop period
    static double timer = target;                  // period we command via setPeriod

    const auto now = std::chrono::steady_clock::now();
    if (last.time_since_epoch().count() != 0)
    {
        const double dt = std::chrono::duration<double, std::milli>(now - last).count();
        if (dt > 0.0 && dt < 10.0 * target)        // ignore startup / stalls
        {
            ema += 0.2 * (dt - ema);               // EMA of the true loop period
            const double err = target - ema;
            // Deadband: once the measured period is within ~1 ms of target, stop
            // adjusting so setPeriod (which logs) is not called at steady state —
            // otherwise it flips ±1 ms every frame and spams the log.
            if (err > 0.75 || err < -0.75)
            {
                timer += 0.5 * err;                // proportional correction
                timer  = std::clamp(timer, 1.0, 1000.0);
                const int p = static_cast<int>(timer + 0.5);
                if (p != getPeriod("Compute"))
                    setPeriod("Compute", p);
            }
        }
    }
    last = now;
}

// ---- Real ZED branch: grab, build the frame, publish/store, then pose. ----
void SpecificWorker::step_real()
{
    if (zed.grab() != sl::ERROR_CODE::SUCCESS)   // no new frame this tick
        return;

    RoboCompCameraRGBDSimple::TRGBD rgbd;
    RoboCompLidar3D::TColorCloudData colorCloud;
    const long num_points = build_real_rgbd(rgbd, colorCloud);
    publish_and_store(std::move(rgbd), colorCloud, num_points);
    process_pose_data();
}

// ---- Simulated branch: pull from the CameraRGBDSimple proxy, reproject depth. ----
void SpecificWorker::step_simulated()
{
    RoboCompCameraRGBDSimple::TRGBD rgbd;
    RoboCompLidar3D::TColorCloudData colorCloud;
    const long num_points = build_simulated_rgbd(rgbd, colorCloud);
    if (num_points < 0)          // no new frame from the source yet
        return;
    publish_and_store(std::move(rgbd), colorCloud, num_points);
}

// Common tail shared by both branches: publish (Ice + DDS) straight from the freshly
// built frame — no lock held during serialization — then move it into the shared
// buffers for the pull servants, and optionally display.
void SpecificWorker::publish_and_store(RoboCompCameraRGBDSimple::TRGBD &&rgbd,
                                       RoboCompLidar3D::TColorCloudData &colorCloud,
                                       long num_points)
{
    if (publish_rgbd)
        camerargbdsimplepub_pubproxy->pushRGBD(rgbd);

    if (publish_dds && dds_publisher)
        publish_dds_frames(rgbd);

    if (compute_color_cloud)
        store_color_cloud(colorCloud, num_points);

    store_rgbd(std::move(rgbd));

    if (display)
        show_frames();
}

// Move the frame into the shared buffer. The (cheap) move-construction runs before
// the lock; only the pointer swap is guarded, minimizing contention with servants.
void SpecificWorker::store_rgbd(RoboCompCameraRGBDSimple::TRGBD &&rgbd)
{
    auto ptr = std::make_shared<RoboCompCameraRGBDSimple::TRGBD>(std::move(rgbd));
    std::unique_lock<std::shared_mutex> lock(rgbd_mutex);
    buffer_rgbd = std::move(ptr);
}

///////////////////////////////////////////////////////////////////////

// Acquire from the real ZED camera and build the RGBD frame (+ optional color cloud).
// RGB is converted RGBA->RGB straight INTO the Ice byte buffer and depth is memcpy'd
// straight in — no intermediate cv::Mat/vector copies. Returns the valid point count.
long SpecificWorker::build_real_rgbd(RoboCompCameraRGBDSimple::TRGBD &rgbd,
                                     RoboCompLidar3D::TColorCloudData &colorCloud)
{
    zed.retrieveImage(image, sl::VIEW::LEFT);
    zed.retrieveMeasure(depth, sl::MEASURE::DEPTH);
    zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);

    const long alivetime = image.timestamp.getNanoseconds();
    colorCloud.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    const int w = image.getWidth();
    const int h = image.getHeight();

    // --- RGB: convert the ZED's native BGRA -> RGB directly into the Ice byte buffer (no copy/alloc) ---
    // sl::VIEW::LEFT retrieves a 4-channel BGRA image, so the swap-and-drop must be BGRA2RGB (not
    // RGBA2RGB, which only strips alpha and would leave the bytes in BGR order under an RGB8 label).
    RoboCompCameraRGBDSimple::TImage &rgb_image = rgbd.image;
    rgb_image.width = w;   rgb_image.height = h;  rgb_image.depth = 3;
    rgb_image.cameraID = 0;
    rgb_image.focalx = cam_params.fx;  rgb_image.focaly = cam_params.fy;
    rgb_image.alivetime = alivetime;   rgb_image.period = init_parameters.camera_fps;
    rgb_image.compressed = false;
    rgb_image.image.resize(static_cast<size_t>(w) * h * 3);
    {
        const cv::Mat bgra(h, w, CV_8UC4, image.getPtr<sl::uchar1>(sl::MEM::CPU));
        cv::Mat rgb_dst(h, w, CV_8UC3, rgb_image.image.data());
        cv::cvtColor(bgra, rgb_dst, cv::COLOR_BGRA2RGB);
    }

    // --- Depth: memcpy the float depth straight into the Ice byte buffer ---
    const int dw = depth.getWidth();
    const int dh = depth.getHeight();
    RoboCompCameraRGBDSimple::TDepth &depth_image = rgbd.depth;
    depth_image.width = dw;  depth_image.height = dh;
    depth_image.cameraID = 0;
    depth_image.focalx = cam_params.fx;  depth_image.focaly = cam_params.fy;
    depth_image.alivetime = alivetime;   depth_image.period = init_parameters.camera_fps;
    depth_image.compressed = false;
    depth_image.depth.resize(static_cast<size_t>(dw) * dh * sizeof(float));
    std::memcpy(depth_image.depth.data(), depth.getPtr<float>(sl::MEM::CPU), depth_image.depth.size());

    // --- Reproject the aligned XYZRGBA cloud into the robot frame ---
    const cv::Mat pc(point_cloud.getHeight(), point_cloud.getWidth(), CV_32FC4,
                     point_cloud.getPtr<sl::float1>(sl::MEM::CPU));
    const Eigen::Matrix3f R = extrinsic.linear();
    const Eigen::Vector3f t = extrinsic.translation();

    std::vector<RoboCompCameraRGBDSimple::Point3D> cloud(pc.total());
    if (compute_color_cloud)
        resize_color_cloud(colorCloud, pc.total());

    long index = 0;
    for (int y = 0; y < pc.rows; ++y)
    {
        const cv::Vec4f *row = pc.ptr<cv::Vec4f>(y);
        for (int x = 0; x < pc.cols; ++x)
        {
            const cv::Vec4f &v = row[x];
            if (!std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2]))
                continue;

            const Eigen::Vector3f p = R * Eigen::Vector3f(v[0], v[1], v[2]) + t;
            cloud[index].x = p.x();
            cloud[index].y = p.y();
            cloud[index].z = p.z();

            if (compute_color_cloud)
            {
                colorCloud.X[index] = p.x();
                colorCloud.Y[index] = p.y();
                colorCloud.Z[index] = p.z();
                const uint32_t rgba = *reinterpret_cast<const uint32_t *>(&v[3]);
                colorCloud.R[index] = (rgba >> 0)  & 0xFF;
                colorCloud.G[index] = (rgba >> 8)  & 0xFF;
                colorCloud.B[index] = (rgba >> 16) & 0xFF;
            }
            ++index;
        }
    }
    cloud.resize(index);

    rgbd.points.alivetime = alivetime;
    rgbd.points.period = init_parameters.camera_fps;
    rgbd.points.compressed = false;
    rgbd.points.points = std::move(cloud);
    return index;
}

// Pull one frame from the simulated CameraRGBDSimple proxy (Webots) and reproject its
// depth into a point cloud. The source image/depth buffers are MOVED into the frame
// (no copy). Returns the valid point count, or -1 when the source has no new frame.
long SpecificWorker::build_simulated_rgbd(RoboCompCameraRGBDSimple::TRGBD &rgbd,
                                          RoboCompLidar3D::TColorCloudData &colorCloud)
{
    // Single round-trip to the source (image + depth) instead of two separate calls.
    RoboCompCameraRGBDSimple::TRGBD src = camerargbdsimple_proxy->getAll("");

    // Advance only on a genuinely new frame (dedup by the source stamp).
    static long last_source_timestamp = -1;
    if (src.depth.alivetime == last_source_timestamp)
        return -1;
    last_source_timestamp = src.depth.alivetime;

    // Move the source buffers into our frame (no image/depth copy).
    rgbd.image = std::move(src.image);
    rgbd.depth = std::move(src.depth);
    const RoboCompCameraRGBDSimple::TImage &rgb_image = rgbd.image;
    const RoboCompCameraRGBDSimple::TDepth &depth_image = rgbd.depth;

    colorCloud.timestamp = depth_image.alivetime;

    const int width  = depth_image.width;
    const int height = depth_image.height;

    std::vector<RoboCompCameraRGBDSimple::Point3D> cloud;
    long index = 0;

    if (compute_xyz && not depth_image.depth.empty() && not rgb_image.image.empty())
    {
        const float fx = depth_image.focalx;
        const float fy = depth_image.focaly;
        const float cx = width  / 2.0f;
        const float cy = height / 2.0f;

        // Precomputed unprojection tables, rebuilt only when the frame size changes.
        static std::vector<float> xTable, yTable;
        if (xTable.size() != (size_t)width || yTable.size() != (size_t)height)
        {
            xTable.resize(width);
            yTable.resize(height);
            for (int u = 0; u < width; ++u)  xTable[u] = (u - cx) / fx;
            for (int v = 0; v < height; ++v) yTable[v] = (v - cy) / fy;
        }

        // Wrap the (moved-in) byte buffers as cv::Mat without copying.
        const cv::Mat depthMat(height, width, CV_32FC1, const_cast<uint8_t *>(depth_image.depth.data()));
        const cv::Mat cv_image(height, width, CV_8UC3,  const_cast<uint8_t *>(rgb_image.image.data()));

        const Eigen::Matrix3f R = extrinsic.linear();
        const Eigen::Vector3f t = extrinsic.translation();

        cloud.resize(static_cast<size_t>(width) * height);
        if (compute_color_cloud)
            resize_color_cloud(colorCloud, static_cast<size_t>(width) * height);

        for (int v = 0; v < height; ++v)
        {
            const float *depth_ptr = depthMat.ptr<float>(v);
            const uint8_t *image_ptr = cv_image.ptr<uint8_t>(v);
            const float yt = yTable[v];
            for (int u = 0; u < width; ++u)
            {
                float d = depth_ptr[u];
                if (d <= 0.0f || d == INFINITY) continue;
                d *= 1000.0f;   // m -> mm

                const float x = xTable[u] * d;
                const float y = yt * d;
                const Eigen::Vector3f p = R * Eigen::Vector3f(x, d, -y) + t;

                cloud[index].x = p.x();
                cloud[index].y = p.y();
                cloud[index].z = p.z();

                if (compute_color_cloud)
                {
                    const uint8_t *px = image_ptr + 3 * u;
                    colorCloud.X[index] = p.x();
                    colorCloud.Y[index] = p.y();
                    colorCloud.Z[index] = p.z();
                    colorCloud.R[index] = px[0];
                    colorCloud.G[index] = px[1];
                    colorCloud.B[index] = px[2];
                }
                ++index;
            }
        }
    }
    cloud.resize(index);

    rgbd.points.alivetime = depth_image.alivetime;
    rgbd.points.period = depth_image.period;
    rgbd.points.compressed = false;
    rgbd.points.points = std::move(cloud);
    return index;
}

// Trim the color cloud arrays to the valid point count and store it into the buffer.
void SpecificWorker::store_color_cloud(RoboCompLidar3D::TColorCloudData &colorCloud, long num_points)
{
    colorCloud.numberPoints = num_points;
    colorCloud.compressed = false;
    resize_color_cloud(colorCloud, num_points);

    auto ptr = std::make_shared<RoboCompLidar3D::TColorCloudData>(std::move(colorCloud));
    std::lock_guard<std::shared_mutex> lock(color_point_cloud_mutex);
    buffer_color_point_cloud = std::move(ptr);
}

// Publish the given RGBD frame on the zero-copy DDS media plane (RGB + depth as
// separate ImageFrame streams). Reads the local frame directly — no lock, no buffer
// deref. The source alivetime is normalized to epoch-ms so consumers can realign.
void SpecificWorker::publish_dds_frames(const RoboCompCameraRGBDSimple::TRGBD &rgbd)
{
    const auto &img = rgbd.image;
    const auto &dep = rgbd.depth;

    // ZED SDK reports epoch ns (~1.7e18); a Webots bridge may already report ms
    // (~1.7e12). Scale down only ns-magnitude values (matches robot_concept / lidar).
    const long long t = img.alivetime;
    const std::uint64_t stamp_ms = t > 1'000'000'000'000'000LL
        ? static_cast<std::uint64_t>(t / 1'000'000)
        : static_cast<std::uint64_t>(t);

    if (!img.image.empty() && img.width > 0 && img.height > 0)
    {
        const std::uint32_t channels = img.depth > 0 ? static_cast<std::uint32_t>(img.depth) : 3u;
        dds_publisher->publish_rgb(stamp_ms,
                                   static_cast<std::uint32_t>(img.width),
                                   static_cast<std::uint32_t>(img.height),
                                   static_cast<std::uint32_t>(img.width) * channels,
                                   img.image.data(), img.image.size());
    }
    if (!dep.depth.empty() && dep.width > 0 && dep.height > 0)
    {
        // depth carries float32 pixels: row stride = width*4, size() = total bytes.
        dds_publisher->publish_depth(stamp_ms,
                                     static_cast<std::uint32_t>(dep.width),
                                     static_cast<std::uint32_t>(dep.height),
                                     static_cast<std::uint32_t>(dep.width) * sizeof(float),
                                     reinterpret_cast<const std::uint8_t *>(dep.depth.data()),
                                     dep.depth.size());
    }
}

// Render the last buffered RGBD frame with OpenCV.
void SpecificWorker::show_frames()
{
    std::shared_lock<std::shared_mutex> lock(rgbd_mutex);
    if (!buffer_rgbd)
        return;

    cv::Mat rgb_frame(cv::Size(buffer_rgbd->image.width, buffer_rgbd->image.height),
                      CV_8UC3, &buffer_rgbd->image.image[0], cv::Mat::AUTO_STEP);
    cv::Mat depth_frame(cv::Size(buffer_rgbd->depth.width, buffer_rgbd->depth.height),
                        CV_32FC1, &buffer_rgbd->depth.depth[0], cv::Mat::AUTO_STEP);

    // Depth scaling differs by source: simulated depth is in metres, real ZED in mm.
    if (simulated)
    {
        cv::cvtColor(rgb_frame, rgb_frame, cv::COLOR_RGB2BGR);
        depth_frame.convertTo(depth_frame, CV_8UC1, 255.0 / 10, 0);
    }
    else
        depth_frame.convertTo(depth_frame, CV_8UC1, 255.0 / 10000, 0);

    cv::applyColorMap(depth_frame, depth_frame, cv::COLORMAP_RAINBOW);
    cv::imshow("rgb", rgb_frame);
    cv::imshow("depth", depth_frame);
    cv::waitKey(1);
}

void SpecificWorker::process_pose_data()
{
    static long last_timestamp = 0;


    // 1) ZED pose
    #ifdef POSE
        zed.getPosition(zed_pose, sl::REFERENCE_FRAME::WORLD);
    #else
        zed.getPosition(zed_pose, sl::REFERENCE_FRAME::CAMERA);
    #endif
    // 2) traslación y orientación
    auto t_cam_sl = zed_pose.getTranslation();
    auto r_cam_sl = zed_pose.getEulerAngles(true);
    Eigen::Vector3f t_cam(t_cam_sl.tx, t_cam_sl.ty, t_cam_sl.tz);
    Eigen::AngleAxisf aa_x(r_cam_sl.x, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf aa_y(r_cam_sl.y, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf aa_z(r_cam_sl.z, Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf q_cam = aa_z * aa_y * aa_x;  // orden ZYX

    // 3) Transformación fija cámara?robot
    static const Eigen::Vector3f cam_to_robot_t(60.0f, 76.6f, 0.0f);

    // 4) Velocidades (diferencial)
    long timestamp = zed_pose.timestamp.getNanoseconds();
    float dt = (timestamp - last_timestamp) * 1e-9f;  // segundos
    last_timestamp = timestamp;
    float* cov = zed_pose.pose_covariance;

    // Print all values
    //std::cout << "Pose covariance: ";
    //for (int i = 0; i < 36; ++i) {
    //    std::cout << cov[i] << " ";
    //    if ((i + 1) % 6 == 0) std::cout << std::endl;
    //}
    RoboCompFullPoseEstimation::CovMatrix pose_cov_matrix{
        .m00 = cov[0], .m01 = cov[1], .m02 = cov[2], .m03 = cov[3], .m04 = cov[4], .m05 = cov[5],
        .m10 = cov[6], .m11 = cov[7], .m12 = cov[8], .m13 = cov[9], .m14 = cov[10], .m15 = cov[11],
        .m20 = cov[12], .m21 = cov[13], .m22 = cov[14], .m23 = cov[15], .m24 = cov[16], .m25 = cov[17],
        .m30 = cov[18], .m31 = cov[19], .m32 = cov[20], .m33 = cov[21], .m34 = cov[22], .m35 = cov[23],
        .m40 = cov[24], .m41 = cov[25], .m42 = cov[26], .m43 = cov[27], .m44 = cov[28], .m45 = cov[29],
        .m50 = cov[30], .m51 = cov[31], .m52 = cov[32], .m53 = cov[33], .m54 = cov[34], .m55 = cov[35]
    };

    Eigen::Vector3f v_cam = t_cam / dt;
    Eigen::Vector3f omega_cam(r_cam_sl.x / dt, r_cam_sl.y / dt, r_cam_sl.z / dt);

    // 6) Corrección de velocidad lineal: v_robot = v_cam + ?_cam × (lever arm)
    Eigen::Vector3f lever = q_cam * cam_to_robot_t;
    Eigen::Vector3f v_robot = v_cam + omega_cam.cross(lever);
    Eigen::Vector3f omega_robot = omega_cam;

    // 7) Publicación
    RoboCompFullPoseEstimation::FullPoseEuler pose;
    pose.x  = t_cam_sl.x;
    pose.y  = t_cam_sl.y;
    pose.z  = t_cam_sl.z;
    pose.rx = r_cam_sl.x;
    pose.ry = r_cam_sl.y;
    pose.rz = r_cam_sl.z;
    pose.vx  = v_robot.x();
    pose.vy  = v_robot.y();
    pose.vz  = v_robot.z();
    pose.vrx = omega_robot.x();
    pose.vry = omega_robot.y();
    pose.vrz = omega_robot.z();
    pose.poseCov = pose_cov_matrix;
    pose.timestamp = zed_pose.timestamp.getMilliseconds();

    try
    {
        ; //fullposeestimationpub_pubproxy->newFullPose(pose); // TODO: enable pose publishing
    }
    catch (const Ice::Exception &e)
    {
        std::cerr << "Error publishing pose: " << e.what() << std::endl;
    }

    //std::cout<<std::setprecision(3)<<"\rX:"<<pose.x<<"|Y:"<<pose.y<<"|Z:"<< pose.z<<
        // "| RX:"<< pose.rx << "|RY:"<<pose.ry<< "|RZ:"<< pose.rz<<"| VX:"<<pose.vx<<"|VY:"<<pose.vy<<"|VZ:"<< pose.vz<<
        // "| VRX:"<< pose.vrx << "|VRY:"<<pose.vry<< "|VRZ:"<< pose.vrz<<"| Time:"<<pose.timestamp<<"              ";    

}

void SpecificWorker::sensorsLoop()
{
    const int N = 16;                   // Downsampling factor (publicarás a 100 Hz si N=4)
    int sample_count = 0;
    // Acumuladores para aceleración y giro
    sl::float3 acc_sum{0,0,0}, gyr_sum{0,0,0};
    while (running)
    {
        // Get start time
        auto start = std::chrono::high_resolution_clock::now();
        zed.getSensorsData(sensors_data, sl::TIME_REFERENCE::CURRENT);

        // Solo consideramos IMU para downsampling
        if (ts.isNew(sensors_data.imu))
        {
            // Acumular muestras
            const auto &imu = sensors_data.imu;
            acc_sum.x += imu.linear_acceleration.x;
            acc_sum.y += imu.linear_acceleration.y;
            acc_sum.z += imu.linear_acceleration.z;
            gyr_sum.x += imu.angular_velocity.x;
            gyr_sum.y += imu.angular_velocity.y;
            gyr_sum.z += imu.angular_velocity.z;
            sample_count++;

            if (sample_count >= N)
            {
                // Calcular promedio
                sl::float3 acc_avg{ acc_sum.x / float(N),
                                    acc_sum.y / float(N),
                                    acc_sum.z / float(N) };
                sl::float3 gyr_avg{ gyr_sum.x / float(N),
                                    gyr_sum.y / float(N),
                                    gyr_sum.z / float(N) };

                // Publicar por proxy
                RoboCompIMU::DataImu imu_data;
                RoboCompIMU::Gyroscope gyro_data;
                gyro_data.XGyr = gyr_avg.x;
                gyro_data.YGyr = gyr_avg.y;
                gyro_data.ZGyr = gyr_avg.z;
                gyro_data.timestamp = sensors_data.imu.timestamp.getNanoseconds();
                RoboCompIMU::Acceleration accel_data;
                accel_data.XAcc = acc_avg.x;
                accel_data.YAcc = acc_avg.y;
                accel_data.ZAcc = acc_avg.z;
                accel_data.timestamp = sensors_data.imu.timestamp.getNanoseconds();
                imu_data.acc  = accel_data;
                imu_data.gyro = gyro_data;

                this->imupub_pubproxy->publish(imu_data);

                // Reiniciar acumuladores
                sample_count = 0;
                acc_sum = {0,0,0};
                gyr_sum = {0,0,0};
            }
        }
        // Publicar magnetómetro y barómetro
        if (ts.isNew(sensors_data.magnetometer))
        {
//            std::cout << " - Magnetometer\n\t Magnetic Field: {"
//                      << sensors_data.magnetometer.magnetic_field_calibrated
//                      << "} [uT]\n";
        }
        if (ts.isNew(sensors_data.barometer))
        {
//            std::cout << " - Barometer\n\t Atmospheric pressure: "
//                      << sensors_data.barometer.pressure << " [hPa]\n";
        }

        // Get end time
        auto end = std::chrono::high_resolution_clock::now();
        // Calculate elapsed time
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        // Sleep until the next iteration taking into account a required loop time
//        std::cout << "Elapsed time: " << elapsed << " microseconds" << std::endl;
//        std::cout << "Sleeping for: " << 2500 - elapsed << " microseconds" << std::endl;
        std::this_thread::sleep_for(std::chrono::microseconds(2500 - elapsed));
    }
}

float SpecificWorker::wrapToPi(float angle_rad) {
    angle_rad = std::fmod(angle_rad + M_PI, 2.0f * M_PI);
    if (angle_rad < 0)
        angle_rad += 2.0f * M_PI;
    return angle_rad - M_PI;
}

//**************************************/AUX/**************************************/
void SpecificWorker::transformPose(sl::Transform &pose, sl::Transform transform) {
    // Pose(new reference frame) = Pose (camera frame) * M, where M is the transform between two frames
    pose = pose * transform;
}

void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}

//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}

///////////////////////////////////////////////////////////////////////////////

RoboCompCameraRGBDSimple::TRGBD SpecificWorker::CameraRGBDSimple_getAll(std::string camera)
{
    std::shared_lock<std::shared_mutex> lock(rgbd_mutex);
    if (buffer_rgbd)
        return *buffer_rgbd; 
    else
        return RoboCompCameraRGBDSimple::TRGBD();
}

RoboCompCameraRGBDSimple::TDepth SpecificWorker::CameraRGBDSimple_getDepth(std::string camera)
{
	std::shared_lock<std::shared_mutex> lock(rgbd_mutex);
    if (buffer_rgbd)
        return buffer_rgbd->depth; 
    else
        return RoboCompCameraRGBDSimple::TDepth();
}

RoboCompCameraRGBDSimple::TImage SpecificWorker::CameraRGBDSimple_getImage(std::string camera)
{
	std::shared_lock<std::shared_mutex> lock(rgbd_mutex);
    if (buffer_rgbd)
        return buffer_rgbd->image; 
    else
        return RoboCompCameraRGBDSimple::TImage();
}

RoboCompCameraRGBDSimple::TPoints SpecificWorker::CameraRGBDSimple_getPoints(std::string camera)
{
	std::shared_lock<std::shared_mutex> lock(rgbd_mutex);
    if (buffer_rgbd)
        return buffer_rgbd->points; 
    else
        return RoboCompCameraRGBDSimple::TPoints();
}

std::string SpecificWorker::MediaPlaneDDS_getMediaDescriptor()
{
    // Report the live DDS media-plane descriptor (JSON) so the robot_concept agent
    // can relay it into DSR. Empty when DDS publishing is disabled/failed, which
    // tells the consumer to fall back to bridging the frames itself.
    if (dds_publisher)
        return dds_publisher->descriptor_json();
    return {};
}

RoboCompLidar3D::TColorCloudData SpecificWorker::Lidar3D_getColorCloudData()
{
    std::shared_lock<std::shared_mutex> lock(color_point_cloud_mutex);
    if (buffer_color_point_cloud)
        return *buffer_color_point_cloud; 
    else
        return RoboCompLidar3D::TColorCloudData();
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarData(std::string name, float start, float len, int decimationDegreeFactor)
{
	RoboCompLidar3D::TData ret{};
	//implementCODE

	return ret;
}

RoboCompLidar3D::TDataImage SpecificWorker::Lidar3D_getLidarDataArrayProyectedInImage(std::string name)
{
	RoboCompLidar3D::TDataImage ret{};
	//implementCODE

	return ret;
}

RoboCompLidar3D::TDataCategory SpecificWorker::Lidar3D_getLidarDataByCategory(RoboCompLidar3D::TCategories categories, Ice::Long timestamp)
{
	RoboCompLidar3D::TDataCategory ret{};
	//implementCODE

	return ret;
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataProyectedInImage(std::string name)
{
	RoboCompLidar3D::TData ret{};
	//implementCODE

	return ret;
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor)
{
	RoboCompLidar3D::TData ret{};
	//implementCODE

	return ret;
}

/**************************************/
// From the RoboCompCameraRGBDSimple you can call this methods:
// RoboCompCameraRGBDSimple::TRGBD this->camerargbdsimple_proxy->getAll(string camera)
// RoboCompCameraRGBDSimple::TDepth this->camerargbdsimple_proxy->getDepth(string camera)
// RoboCompCameraRGBDSimple::TImage this->camerargbdsimple_proxy->getImage(string camera)
// RoboCompCameraRGBDSimple::TPoints this->camerargbdsimple_proxy->getPoints(string camera)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::Point3D
// RoboCompCameraRGBDSimple::TPoints
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompCameraRGBDSimplePub you can publish calling this methods:
// RoboCompCameraRGBDSimplePub::void this->camerargbdsimplepub_pubproxy->pushRGBD(RoboCompCameraRGBDSimple::TRGBD rgbd)

/**************************************/
// From the RoboCompFullPoseEstimationPub you can publish calling this methods:
// RoboCompFullPoseEstimationPub::void this->fullposeestimationpub_pubproxy->newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose)

/**************************************/
// From the RoboCompIMUPub you can publish calling this methods:
// RoboCompIMUPub::void this->imupub_pubproxy->publish(RoboCompIMU::DataImu imu)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::Point3D
// RoboCompCameraRGBDSimple::TPoints
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData

