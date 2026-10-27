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
#include "dds_publisher.h"
#include <sys/stat.h>
#include <unistd.h>
#include <cstdlib>
#include <csignal>
#include <dirent.h>
#include <fcntl.h>
#include <regex>
#include <fstream>
#include <sstream>

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


		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
		
	}
}
/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	if (mediamtx_pid > 0)
		::kill(static_cast<pid_t>(mediamtx_pid), SIGTERM);
	if (gpu_helper_pid > 0)
		::kill(static_cast<pid_t>(gpu_helper_pid), SIGTERM);
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::reconnect_camera()
{
    std::cout << "Attempting camera reconnection..." << std::endl;

    capture.release();

    // GPU decode runs out-of-process; if the helper itself died (e.g. the h264parse
    // segfault — see debug_ricoh_omni_dds.md), respawn it before touching our own
    // shmsrc capture pipeline, otherwise we'd just be reconnecting to a dead socket.
    if(pars.use_gpu && !gpu_helper_alive())
    {
        std::cout << "GPU decode helper is down, respawning..." << std::endl;
        launch_gpu_helper(); // blocks until the shm socket reappears, refreshes gpu_caps_fragment
        pipeline = "shmsrc socket-path=" + pars.gpu_shm_socket + " ! " + gpu_caps_fragment + " ! appsink max-buffers=1 drop=true sync=false";
    }

    // Pequeña pausa para que el dispositivo USB se libere
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if(capture.open(pipeline, cv::CAP_GSTREAMER))
    {
        // Verify pipeline is functional before trusting open()
        try {
            if(!capture.grab()) {
                qWarning() << "Reconnect: pipeline opened but grab() failed";
                capture.release();
                activated_camera = false;
                return false;
            }
        } catch(const cv::Exception& e) {
            qWarning() << "Reconnect: grab() threw:" << e.what();
            capture.release();
            activated_camera = false;
            return false;
        }

        // Esperar a que el decoder estabilice el stream
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Intentar grab un par de frames (los primeros pueden ser corruptos)
        for(int i = 0; i < 3; i++)
        {
            if(capture.grab())
            {
                capture.retrieve(cv_frame);
                if(!cv_frame.empty())
                {
                    MAX_WIDTH = cv_frame.cols;
                    MAX_HEIGHT = cv_frame.rows;
                    activated_camera = true;
                    std::cout << "Camera reconnected! Resolution: " << MAX_WIDTH << "x" << MAX_HEIGHT << std::endl;

                    // Reconectar streaming si estaba habilitado
                    if(pars.streaming && !pars.simulator && !streaming_enabled)
                    {
                        // Defensive close: if the previous streaming pipeline is still
                        // open (e.g. release() couldn't fully join it because rtmpsink
                        // was stuck on a stalled write), reopening on top of it was what
                        // leaked a full encode pipeline + its threads on every reconnect.
                        if(stream_writer.isOpened())
                            stream_writer.release();
                        stream_write_errors = 0;

                        try {
                            if(stream_writer.open(stream_pipeline, cv::CAP_GSTREAMER, 0, 30,
                                                 cv::Size(MAX_WIDTH, MAX_HEIGHT), true))
                            {
                                streaming_enabled = true;
                                last_stream_refresh = std::chrono::steady_clock::now();
                                std::cout << "Streaming pipeline reconnected!" << std::endl;
                            }
                        } catch(...) {}
                    }

                    return true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        qWarning() << "Camera opened but failed to grab frame";
        capture.release();
    }
    else
    {
        qWarning() << "Failed to reopen camera pipeline";
    }

    activated_camera = false;
    return false;
}

// Rate-limits reconnect_camera() with an adaptive backoff instead of the fixed 2s
// cooldown: a flapping USB link (xhci_hcd errors every 10-30s) was retrying every 2s
// forever, and each attempt can leave behind a GStreamer pipeline that release()
// didn't fully tear down (source/sink stuck mid-teardown by the same USB glitch) —
// leaking its threads. Backing off on repeated failures slows that leak rate; a clean
// success resets it back to the fast 2s cooldown.
void SpecificWorker::maybe_reconnect()
{
    auto now = std::chrono::steady_clock::now();
    if(std::chrono::duration_cast<std::chrono::seconds>(now - last_reconnect).count() < reconnect_cooldown_s)
        return;

    last_reconnect = now;
    if(reconnect_camera())
        reconnect_cooldown_s = 2;
    else
        reconnect_cooldown_s = std::min(reconnect_cooldown_s * 2, MAX_RECONNECT_COOLDOWN_S);
}

void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;

    // Some launch contexts (systemd, screen/tmux without a full login session) don't
    // export XDG_RUNTIME_DIR, which the GL-based decode pipeline (gldownload) needs
    // for its EGL context. Fall back to a private runtime dir instead of failing.
    if (!qEnvironmentVariableIsSet("XDG_RUNTIME_DIR"))
    {
        std::string runtime_dir = "/tmp/ricoh_omni_dds-runtime-" + std::to_string(getuid());
        mkdir(runtime_dir.c_str(), 0700);
        setenv("XDG_RUNTIME_DIR", runtime_dir.c_str(), 1);
        std::cout << "XDG_RUNTIME_DIR was not set, using " << runtime_dir << std::endl;
    }

	GenericWorker::initialize();

    try
    {
        //Parametros por el config
        pars.display = this->configLoader.get<bool>("display");
        pars.simulator = this->configLoader.get<bool>("simulator");
        pars.orin = this->configLoader.get<bool>("orin");
        pars.use_gpu = this->configLoader.get<bool>("use_gpu");

        // GStreamer NVDEC element name varies across nvcodec plugin builds/versions
        // (newer per-codec "nvh264dec" vs older generic "nvdec") — configurable per machine.
        try {
            pars.gpu_decoder = this->configLoader.get<std::string>("gpu_decoder");
        } catch(...) {
            pars.gpu_decoder = "nvh264dec";
        }

        // avdec_h264/x264enc default to 0 ("auto" = one thread per CPU core), which on
        // a high core-count machine alone can push CPU into the hundreds of percent —
        // configurable per machine since core counts differ (shadow-hp vs p3bot).
        try {
            pars.decode_threads = this->configLoader.get<int>("decode_threads");
        } catch(...) {
            pars.decode_threads = 4;
        }

        try {
            pars.encode_threads = this->configLoader.get<int>("encode_threads");
        } catch(...) {
            pars.encode_threads = 2;
        }

        try {
            pars.gpu_downloader = this->configLoader.get<std::string>("gpu_downloader");
        } catch(...) {
            pars.gpu_downloader = "gldownload";
        }

        try {
            pars.gpu_shm_socket = this->configLoader.get<std::string>("gpu_shm_socket");
        } catch(...) {
            pars.gpu_shm_socket = "/tmp/ricoh_omni_dds_gpu.sock";
        }

        try {
            pars.gpu_shm_size = this->configLoader.get<int>("gpu_shm_size");
        } catch(...) {
            pars.gpu_shm_size = 100000000;
        }
        pars.compressed = this->configLoader.get<bool>("compressed");
        pars.time_offset = this->configLoader.get<double>("time_offset");

        // Parámetros de streaming (opcionales, por defecto activado)
        try {
            pars.streaming = this->configLoader.get<bool>("streaming");
        } catch(...) {
            pars.streaming = true; // Por defecto habilitado
        }

        try {
            pars.stream_host = this->configLoader.get<std::string>("stream_host");
        } catch(...) {
            pars.stream_host = "localhost";
        }

        try {
            pars.stream_port = this->configLoader.get<int>("stream_port");
        } catch(...) {
            pars.stream_port = 8554;
        }

        try {
            pars.stream_path = this->configLoader.get<std::string>("stream_path");
        } catch(...) {
            pars.stream_path = "theta";
        }

        // MediaMTX gestionado por el propio componente (opcional, activado por defecto)
        try {
            pars.launch_mediamtx = this->configLoader.get<bool>("launch_mediamtx");
        } catch(...) {
            pars.launch_mediamtx = true;
        }

        try {
            pars.mediamtx_bin = this->configLoader.get<std::string>("mediamtx_bin");
        } catch(...) {
            pars.mediamtx_bin = "./mediamtx";
        }

        try {
            pars.mediamtx_config = this->configLoader.get<std::string>("mediamtx_config");
        } catch(...) {
            pars.mediamtx_config = "mediamtx.yml";
        }

        std::cout << "Params: device" << pars.device << " display " << pars.display << " compressed: " << pars.compressed
                  << " simulator: " << pars.simulator << " streaming: " << pars.streaming << " offset "<< pars.time_offset <<std::endl;
    }
    catch(const std::exception &e)
    { std::cout << e.what() << " Error reading config params" << std::endl;};

    // ---- Optional DDS 360 media plane (Image360Frame into the CORTEX stack) ----
    // Gated by PublishDDS. Metadata is advertised into DSR by robot_concept, not here.
    try { publish_dds = this->configLoader.get<bool>("PublishDDS"); }
    catch(...) { publish_dds = false; }
    if (publish_dds)
    {
        Ricoh360DDSPublisher::Config dcfg;   // defaults: domain 7, rc/ricoh/rgb, BGR
        try { dcfg.domain_id = static_cast<std::uint32_t>(this->configLoader.get<int>("DDS.Domain")); } catch(...) {}
        try { dcfg.topic = this->configLoader.get<std::string>("DDS.Topic"); } catch(...) {}
        try { dcfg.history_depth = this->configLoader.get<int>("DDS.HistoryDepth"); } catch(...) {}
        try { dcfg.shared_memory_only = this->configLoader.get<bool>("DDS.SharedMemoryOnly"); } catch(...) {}
        try { dcfg.data_sharing = this->configLoader.get<bool>("DDS.DataSharing"); } catch(...) {}
        try { const std::string f = this->configLoader.get<std::string>("DDS.Format"); dcfg.is_bgr = (f != "rgb" && f != "RGB"); } catch(...) {}

        dds_publisher = std::make_unique<Ricoh360DDSPublisher>();
        if (not dds_publisher->init(dcfg))
        {
            std::cerr << "DDS 360 media plane failed to initialize; disabling DDS publishing." << std::endl;
            dds_publisher.reset();
            publish_dds = false;
        }
    }

//chekpoint robocompUpdater
    std::cout << "Initializing worker" << std::endl;

	if(this->startup_check_flag)
		this->startup_check();
	else
    {
        last_read.store(std::chrono::high_resolution_clock::now());

        // MediaMTX solo tiene sentido con cámara real (en modo simulador el frame
        // ya viene de Webots vía proxy Camera360RGB, no de este pipeline).
        if(!pars.simulator && pars.launch_mediamtx)
            launch_mediamtx();

        std::cout << "Configuring streaming pipeline..." << std::endl;

        // ==========================================
        // CONFIGURACIÓN DEL PIPELINE DE ENTRADA (CAPTURA)
        // ==========================================
        std::cout << "Configuring input capture pipeline..." << std::endl;

        std::cout << "Configuring input capture pipeline..." << std::endl;

        if (pars.orin)
        {
            // Orin Hardware Decoder 
            pipeline = "thetauvcsrc mode=2K ! queue leaky=downstream max-size-buffers=1 ! h264parse ! nvv4l2decoder ! nvvidconv ! videoconvert ! video/x-raw,format=BGR ! appsink max-buffers=1 drop=true sync=false";
        }
        else if (pars.use_gpu)
        {
            // PC NVIDIA GPU Hardware Decoder, isolated in its own gst-launch-1.0 child
            // process (element name is machine-specific, see gpu_decoder). h264parse's
            // segfault (see debug_ricoh_omni_dds.md) then only kills the helper, which
            // reconnect_camera() detects and respawns, instead of taking down this
            // component's ICE adapters and DDS publisher with it.
            //
            // Bonus: since the helper negotiates nvh264dec's context via gst-launch-1.0
            // (a real GStreamer app pumping its own bus) instead of through OpenCV's
            // cv::VideoCapture — which never answered nvh264dec's NEED_CONTEXT messages
            // and hung indefinitely — this also sidesteps the GPU hang previously parked
            // on shadow-hp. Our own capture pipeline below is plain shmsrc, no GL/CUDA
            // context involved.
            // wait-for-connection left at its default (true): our shmsrc reader always
            // connects shortly after this helper starts (see launch_gpu_helper()'s
            // socket-poll), so the sink should hold off producing until then. Setting it
            // false made the sink free-run before any reader joined, and a shmsrc that
            // connects mid-stream instead of at the handshake never synced — that's what
            // was hanging capture.grab() forever with no error.
            gpu_helper_pipeline = "thetauvcsrc mode=2K ! queue leaky=downstream max-size-buffers=1 ! h264parse ! " + pars.gpu_decoder + " ! " + pars.gpu_downloader +
                                  " ! queue leaky=downstream max-size-buffers=1 ! videoconvert ! video/x-raw,format=BGR ! shmsink socket-path=" + pars.gpu_shm_socket +
                                  " shm-size=" + std::to_string(pars.gpu_shm_size);

            launch_gpu_helper(); // also runs discover_gpu_caps(), filling gpu_caps_fragment

            // shmsrc alone negotiates caps=ANY: without a *fully specified* capsfilter
            // here (format+width+height), OpenCV's gst_video_info_from_caps() fails
            // ("can't handle unknown layout") or, with no filter at all, samples arrive
            // with no caps at all ("gst_sample_get_caps() returns NULL") — hence using
            // the actual negotiated resolution discovered from the helper's own log.
            pipeline = "shmsrc socket-path=" + pars.gpu_shm_socket + " ! " + gpu_caps_fragment + " ! appsink max-buffers=1 drop=true sync=false";
        }
        else
        {
            // CPU Decoder. No h264parse: avdec_h264 accepts thetauvcsrc's raw byte-stream
            // directly, and h264parse (via GstBaseParse::gst_base_parse_set_latency) is a
            // reproducible segfault source under real running conditions on this camera.
            // max-threads pinned: avdec_h264 defaults to one worker thread per CPU core,
            // which alone drove CPU into the hundreds of percent on a 32-core machine.
            pipeline = "thetauvcsrc mode=2K ! queue leaky=downstream max-size-buffers=1 ! avdec_h264 max-threads=" + std::to_string(pars.decode_threads) +
                       " ! queue leaky=downstream max-size-buffers=1 ! videoconvert n-threads=4 ! video/x-raw,format=BGR ! appsink max-buffers=1 drop=true sync=false";
        }
        
        std::cout << "Stream pipeline: " << stream_pipeline << std::endl;

        int connection_attempts = 0;
        const int max_attempts = 10;

        while(!activated_camera && connection_attempts < max_attempts)
        {
            connection_attempts++;
            std::cout << "Connecting to Camera360RGB (attempt " << connection_attempts << "/" << max_attempts << ")" << std::endl;

            if(!pars.simulator)
            {
                std::cout << "Pipeline: " << pipeline << std::endl;

                if( auto success = capture.open(pipeline, cv::CAP_GSTREAMER);
                        success != true)
                {
                    qWarning() << __FUNCTION__ << " Error connecting. No camera found or pipeline error.";
                    qWarning() << "Make sure:";
                    qWarning() << "  1. Camera is connected via USB";
                    qWarning() << "  2. GST_PLUGIN_PATH is set: export GST_PLUGIN_PATH=/home/robocomp/software/gstthetauvc/thetauvc";
                    qWarning() << "  3. Plugin thetauvcsrc is available: gst-inspect-1.0 thetauvcsrc";

                    std::this_thread::sleep_for(std::chrono::seconds(2));
                }
                else
                {
                    // Verify the pipeline is actually functional before trusting open()
                    try {
                        if(!capture.grab()) {
                            qWarning() << "Pipeline opened but grab() failed — pipeline non-functional";
                            capture.release();
                            std::this_thread::sleep_for(std::chrono::seconds(2));
                            continue;
                        }
                    } catch(const cv::Exception& e) {
                        qWarning() << "Pipeline opened but grab() threw exception:" << e.what();
                        capture.release();
                        std::this_thread::sleep_for(std::chrono::seconds(2));
                        continue;
                    }

                    std::cout << "Camera opened successfully, grabbing first frame..." << std::endl;
                    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
                    compression_params.push_back(50);

                    // Try to grab first frame with timeout
                    bool frame_grabbed = false;
                    for(int i = 0; i < 5; i++)
                    {
                        if(capture.grab())
                        {
                            capture.retrieve(cv_frame);
                            if(!cv_frame.empty())
                            {
                                frame_grabbed = true;
                                break;
                            }
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    }

                    if(frame_grabbed && !cv_frame.empty())
                    {
                        MAX_WIDTH = cv_frame.cols;
                        MAX_HEIGHT = cv_frame.rows;
                        activated_camera = true;
                        std::cout << "Camera activated successfully! Resolution: " << MAX_WIDTH << "x" << MAX_HEIGHT << std::endl;
                    }
                    else
                    {
                        qWarning() << "Failed to grab first frame from camera";
                        capture.release();
                        std::this_thread::sleep_for(std::chrono::seconds(2));
                    }
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
                catch(const Ice::Exception &e)
                {
                    std::cout << "Simulator connection error: " << e.what() << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
        }

        if(!activated_camera)
        {
            qFatal("Failed to activate camera after %d attempts. Exiting.", max_attempts);
        }

        // Inicializar pipeline de streaming paralelo a MediaMTX
        if(pars.streaming && !pars.simulator)
        {
            std::cout << "Initializing parallel streaming to MediaMTX..." << std::endl;

            // Pipeline GStreamer para publicar vía RTSP a MediaMTX
            // shmsink para compartir frames con un proceso GStreamer externo
            // appsrc -> flvmux -> rtmpsink (RTMP es más compatible)
            // threads pinned: x264enc defaults to one worker thread per CPU core, and
            // under tune=zerolatency more threads mainly adds latency without much
            // throughput gain at this bitrate/resolution — keep it low on both counts.
            stream_pipeline = "appsrc ! videoconvert ! video/x-raw,format=I420 ! "
                            "x264enc threads=" + std::to_string(pars.encode_threads) +
                            " tune=zerolatency bitrate=5000 speed-preset=ultrafast key-int-max=30 ! "
                            "video/x-h264,profile=baseline ! "
                            "flvmux streamable=true ! "
                            "rtmpsink location=rtmp://" + pars.stream_host + ":1935/" + pars.stream_path + " sync=false";

            std::cout << "Stream pipeline: " << stream_pipeline << std::endl;

            // Abrir el writer (sin bloquear si falla)
            try {
                if(stream_writer.open(stream_pipeline, cv::CAP_GSTREAMER, 0, 30,
                                     cv::Size(MAX_WIDTH, MAX_HEIGHT), true))
                {
                    streaming_enabled = true;
                    last_stream_refresh = std::chrono::steady_clock::now();
                    std::cout << "✓ Streaming pipeline initialized successfully!" << std::endl;
                    std::cout << "  Publishing to: rtsp://" << pars.stream_host << ":"
                             << pars.stream_port << "/" << pars.stream_path << std::endl;
                }
                else
                {
                    std::cout << "⚠ Could not initialize streaming pipeline. Continuing without streaming." << std::endl;
                    streaming_enabled = false;
                }
            }
            catch(const cv::Exception& e)
            {
                std::cout << "⚠ Streaming initialization failed: " << e.what() << std::endl;
                std::cout << "  Continuing without streaming..." << std::endl;
                streaming_enabled = false;
            }
        }
        else
        {
            std::cout << "Streaming disabled (simulator mode or config)" << std::endl;
            streaming_enabled = false;
        }

        capture_time = -1;
        DEPTH = cv_frame.elemSize();
        setPeriod("Compute", 33);
        
    }
}
void SpecificWorker::compute()
{
    // OPTIMIZADO - Sin pausas, captura continua

    bool frame_valid = false;

    if(not pars.simulator)
    {
        // Real camera capture - optimizado para velocidad
        try
        {
            if(capture.isOpened())
            {
                // read() directo es más rápido que grab()+retrieve()
                const auto t_before_read = steady_clock::now();
                if(!capture.read(cv_frame) || cv_frame.empty())
                {
                    // Camera open but no valid frame - attempt reconnection with backoff
                    maybe_reconnect();
                }
                else
                {
                    // TEST 2026-09-07: capture_time below is system_clock::now() taken AFTER read()
                    // returns -- i.e. after cv::VideoCapture (USB transfer + internal driver/codec
                    // decode) already produced the frame, not at the sensor's true exposure instant.
                    // This measures only how long THIS call blocked, so it is a LOWER BOUND on how
                    // stale capture_time already is -- it cannot see latency accrued inside the
                    // camera/USB stack before the call started. time_offset (etc/config_shadow_real.toml)
                    // is 0.0, uncalibrated -- this number is a first, rough estimate of what it should be.
                    const double read_block_ms = duration<double, std::milli>(
                        steady_clock::now() - t_before_read).count();
                    static double read_block_sum = 0.0, read_block_max = 0.0;
                    static int read_block_n = 0;
                    static auto last_log = steady_clock::now();
                    read_block_sum += read_block_ms;
                    read_block_max = std::max(read_block_max, read_block_ms);
                    ++read_block_n;
                    if (duration<double>(steady_clock::now() - last_log).count() >= 5.0)
                    {
                        std::cout << "[CaptureLatencyDiag] capture.read() blocked avg="
                                  << (read_block_sum / read_block_n) << "ms max=" << read_block_max
                                  << "ms over " << read_block_n << " frames -- LOWER BOUND on"
                                  << " capture_time staleness (configured time_offset="
                                  << pars.time_offset << "ms, live value)" << std::endl;
                        read_block_sum = 0.0; read_block_max = 0.0; read_block_n = 0;
                        last_log = steady_clock::now();
                    }

                    // FIX 2026-09-07: subtract the JUST-MEASURED read_block_ms (this frame's own
                    // capture.read() blocking time) in addition to the static time_offset. read_block_ms
                    // is a per-frame, load-dependent estimate of the read() latency; a static offset
                    // alone cannot track that, only whatever fixed component is left (sensor exposure-
                    // to-readout time, which read_block_ms structurally cannot see -- see the
                    // CaptureLatencyDiag comment above). Still a LOWER BOUND, not a full fix; the
                    // residual fixed bias needs a real physical calibration (LED/clock sync) to settle
                    // time_offset properly.
                    capture_time = duration_cast<milliseconds>(
                        system_clock::now().time_since_epoch()).count()
                        - pars.time_offset - static_cast<int64_t>(read_block_ms);
                    frame_valid = true;
                }
            }
            else
            {
                // Camera not opened - attempt reconnection with backoff
                maybe_reconnect();
            }
        }
        catch(const cv::Exception& e)
        {
            qWarning() << "OpenCV exception:" << e.what();
            maybe_reconnect();
        }
        catch(...)
        {
            qWarning() << "Unknown exception during capture";
            maybe_reconnect();
        }
    }
    else        // Simulator
    {
        try
        {
            image = this->camera360rgb_proxy->getROI(-1, -1, -1, -1, -1, -1);
            cv_frame = cv::Mat(cv::Size(image.width, image.height), CV_8UC3, &image.image[0]);

            if(!cv_frame.empty())
            {
                capture_time = image.timestamp;
                frame_valid = true;
            }
        }
        catch (const Ice::Exception &e)
        {
            qWarning() << "Simulator error:" << e.what();
        }
    }

    // Procesamiento mínimo para máxima velocidad
    if(frame_valid)
    {
        // DDS FIRST (2026-09-11): must never be gated behind streaming/display, which are
        // independent consumer paths with their own (much slower) GStreamer/Qt costs.
        if(publish_dds && dds_publisher && !cv_frame.empty())
        {
            try
            {
                dds_publisher->publish(static_cast<std::uint64_t>(capture_time),
                                       static_cast<std::uint32_t>(cv_frame.cols),
                                       static_cast<std::uint32_t>(cv_frame.rows),
                                       static_cast<std::uint32_t>(cv_frame.step),
                                       cv_frame.data,
                                       cv_frame.total() * cv_frame.elemSize());
            }
            catch(const std::exception& e)
            {
                qWarning() << "DDS publish exception:" << e.what();
            }
        }

        // Publicar frame al stream de MediaMTX (en paralelo, sin bloquear)
        if(pars.streaming && !pars.simulator)
        {
            // OpenCV's GStreamer VideoWriter backend can't report a push failure to
            // application code (writeFrame() only logs a warning — no exception, no
            // return value), so a stalled RTMP link that mediamtx eventually drops
            // (timeout/EOF on its side) is otherwise invisible here forever. This also
            // covers the "100 consecutive errors" disable path below: once that flips
            // streaming_enabled false, nothing else in this component was reopening it
            // (reconnect_camera() only does on a *capture*-side reconnect) — checking
            // regardless of streaming_enabled means a disabled connection gets retried
            // too, not just a healthy one refreshed.
            //
            // Checked against mediamtx's own API instead of blindly tearing down and
            // reopening on a fixed timer: the earlier unconditional-refresh version
            // closed a perfectly healthy connection every single cycle, producing a
            // visible periodic glitch in the remote view (confirmed live,
            // robolab-shadow 2026-09-24). Polling is cheap (local HTTP call every 5s),
            // so only reopen when mediamtx itself says there's no active publisher.
            auto now = std::chrono::steady_clock::now();
            if(std::chrono::duration_cast<std::chrono::seconds>(now - last_stream_refresh).count() >= STREAM_HEALTH_CHECK_INTERVAL_S)
            {
                last_stream_refresh = now;
                if(!streaming_enabled || !stream_writer.isOpened() || !mediamtx_path_has_publisher())
                {
                    if(stream_writer.isOpened())
                        stream_writer.release();
                    streaming_enabled = false;
                    try {
                        if(stream_writer.open(stream_pipeline, cv::CAP_GSTREAMER, 0, 30, cv::Size(MAX_WIDTH, MAX_HEIGHT), true))
                        {
                            streaming_enabled = true;
                            stream_write_errors = 0;
                        }
                    } catch(...) {}
                }
            }

            if(streaming_enabled && stream_writer.isOpened())
            {
                try {
                    // Escribir frame sin bloquear el loop principal
                    stream_writer.write(cv_frame);
                    stream_write_errors = 0; // healthy write: don't carry old failures forward
                }
                catch(const cv::Exception& e) {
                    // Silenciar errores de streaming para no ralentizar captura
                    // Si falla consistentemente, deshabilitamos streaming
                    if(++stream_write_errors > 100) {
                        std::cout << "⚠ Too many streaming errors, disabling..." << std::endl;
                        streaming_enabled = false;
                        stream_writer.release();
                    }
                }
            }
        }

        // Display optimizado (solo si está habilitado)
        if(pars.display)
        {
            try
            {
                // INTER_NEAREST es el más rápido (sin interpolación)
                cv::Mat display_frame;
                cv::resize(cv_frame, display_frame,
                          cv::Size(cv_frame.cols / 2, cv_frame.rows / 2),
                          0, 0, cv::INTER_NEAREST);
                cv::imshow("Ricoh Theta Z1 [REALTIME]", display_frame);
                cv::waitKey(1);
            }
            catch(const cv::Exception& e)
            {
                // Silenciar errores de display para no ralentizar
            }
        }

        // Buffer actualizado (sin clone para máxima velocidad - usar con cuidado)
        // NOTA: Si hay problemas de thread-safety, volver a usar clone()
        buffer_image.put(std::move(cv_frame));

        // Print FPS statistics
        fps.print("FPS:");
    }
    else
    {
        qWarning() << "Skipping frame due to invalid data";
    }
}


// Sets close-on-exec on every currently open fd (ICE adapters' listening sockets,
// FastDDS sockets, etc.) so none of them leak into a forked child. Without this,
// mediamtx would inherit e.g. the Camera360RGB ICE socket and keep it bound even
// after this process exits, causing "Address already in use" on the next run.
static void close_exec_inherited_fds()
{
    DIR *d = opendir("/proc/self/fd");
    if (!d)
        return;
    int dfd = dirfd(d);
    struct dirent *entry;
    while ((entry = readdir(d)) != nullptr)
    {
        char *end;
        long fd = strtol(entry->d_name, &end, 10);
        if (*end != '\0' || fd < 3 || fd == dfd)
            continue;
        int flags = fcntl(static_cast<int>(fd), F_GETFD);
        if (flags != -1)
            fcntl(static_cast<int>(fd), F_SETFD, flags | FD_CLOEXEC);
    }
    closedir(d);
}

void SpecificWorker::launch_mediamtx()
{
    close_exec_inherited_fds();

    // Avoid double-launch if an instance is already running (e.g. left over from a
    // previous crashed run or started manually via start_system.sh).
    QProcess check;
    check.start("pgrep", QStringList{"-x", "mediamtx"});
    check.waitForFinished(1000);
    if (check.exitCode() == 0)
    {
        std::cout << "mediamtx already running, skipping launch" << std::endl;
        return;
    }

    // Launched detached (via a shell, so mediamtx's own stdout/stderr can be
    // redirected to a log file) rather than as a QProcess instance: the camera
    // connection-retry loop below blocks the Qt event loop for several seconds,
    // and a still-running QProcess gets killed on destruction if the event loop
    // never pumped its child-exit notification.
    const QString shell_cmd = QString("exec %1 %2 > /tmp/mediamtx.log 2>&1")
        .arg(QString::fromStdString(pars.mediamtx_bin))
        .arg(QString::fromStdString(pars.mediamtx_config));

    if (!QProcess::startDetached("/bin/sh", {"-c", shell_cmd}, QString(), &mediamtx_pid))
    {
        std::cout << "⚠ Could not launch mediamtx (" << pars.mediamtx_bin
                   << "). Continuing without local streaming server." << std::endl;
        mediamtx_pid = -1;
        return;
    }

    std::cout << "✓ mediamtx launched (PID " << mediamtx_pid
               << "), log: /tmp/mediamtx.log" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
}

// Queries mediamtx's own HTTP API (requires `api: yes` in mediamtx.yml — mediamtx is
// always launched by this same component, on the same host, so the address is fixed
// rather than configurable) for whether our RTMP path currently has an active
// publisher. Used instead of guessing from OpenCV, which can't tell us (see the
// member comment in the header). Defaults to "assume healthy" on any failure to reach
// the API (timeout, API not enabled, mediamtx not up yet) — better to skip a refresh
// than to tear down a connection we have no actual evidence is broken.
bool SpecificWorker::mediamtx_path_has_publisher() const
{
    QProcess curl;
    curl.start("curl", QStringList{"-s", "--max-time", "2",
        QString("http://127.0.0.1:9997/v3/paths/get/%1").arg(QString::fromStdString(pars.stream_path))});
    curl.waitForFinished(3000);
    if (curl.exitCode() != 0)
        return true;

    const QString out = QString::fromUtf8(curl.readAllStandardOutput());
    return out.contains("\"ready\":true");
}

bool SpecificWorker::gpu_helper_alive() const
{
    return gpu_helper_pid > 0 && ::kill(static_cast<pid_t>(gpu_helper_pid), 0) == 0;
}

void SpecificWorker::launch_gpu_helper()
{
    close_exec_inherited_fds();

    if (gpu_helper_alive())
    {
        std::cout << "GPU decode helper already running (PID " << gpu_helper_pid << ")" << std::endl;
        return;
    }

    // gpu_helper_pid only tracks a helper *this process instance* launched — it resets
    // to -1 on every fresh start, so it's blind to a helper orphaned by a previous run
    // (crash, kill -9, etc.). That orphan is a detached, independent process: it keeps
    // the Theta's USB video interface claimed exclusively, so a second thetauvcsrc
    // instance launched on top of it can't get the device and just hangs — which is
    // what silently stalled capture.grab() forever with no error message. Kill anything
    // still bound to our own shm socket path before starting a fresh one.
    QProcess pkill;
    pkill.start("pkill", QStringList{"-f", "shmsink socket-path=" + QString::fromStdString(pars.gpu_shm_socket)});
    pkill.waitForFinished(1000);
    if (pkill.exitCode() == 0)
    {
        std::cout << "Killed a stale GPU decode helper from a previous run" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // let it release the USB device
    }

    // A stale socket from a previous crashed run would make shmsink fail to bind.
    ::unlink(pars.gpu_shm_socket.c_str());

    // Same detached-via-shell pattern as launch_mediamtx(): a plain gst-launch-1.0
    // process running the GPU pipeline standalone. Being a real GStreamer app pumping
    // its own bus, it negotiates nvh264dec's context correctly (unlike cv::VideoCapture
    // — see the note in initialize()); if h264parse segfaults, only this process dies.
    // -v: verbose pad-caps output, parsed by discover_gpu_caps() below.
    // --no-fault: gst-launch-1.0 installs its OWN SIGSEGV handler by default, which
    // tries to exec gdb and then spins forever waiting for a debugger to attach instead
    // of actually dying. That defeated the whole point of this isolation: the process
    // stayed "alive" per kill(pid,0), so gpu_helper_alive() never saw it as dead and
    // reconnect_camera() never respawned it — the h264parse segfault just hung the
    // stream forever instead of self-healing. Confirmed live on shadow-hp 2026-08-13.
    const QString shell_cmd = QString("exec gst-launch-1.0 --no-fault -v %1 > /tmp/ricoh_gpu_helper.log 2>&1")
        .arg(QString::fromStdString(gpu_helper_pipeline));

    if (!QProcess::startDetached("/bin/sh", {"-c", shell_cmd}, QString(), &gpu_helper_pid))
    {
        std::cout << "⚠ Could not launch GPU decode helper process." << std::endl;
        gpu_helper_pid = -1;
        return;
    }

    std::cout << "✓ GPU decode helper launched (PID " << gpu_helper_pid
               << "), log: /tmp/ricoh_gpu_helper.log" << std::endl;

    // Wait for shmsink to actually bind the socket rather than sleeping a fixed guess:
    // thetauvcsrc enumeration + h264 decoder/context warmup in the child's own pipeline
    // is not a fixed-duration step, so a blind sleep here was either too short (our own
    // capture.open() below racing an ENOENT) or wastefully long.
    struct stat st{};
    for (int i = 0; i < 50 && stat(pars.gpu_shm_socket.c_str(), &st) != 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (stat(pars.gpu_shm_socket.c_str(), &st) != 0)
    {
        qWarning() << "GPU decode helper socket did not appear within 5s:" << pars.gpu_shm_socket.c_str();
        return;
    }

    if (!discover_gpu_caps())
        qWarning() << "Could not read the GPU helper's negotiated caps in time; "
                       "falling back to format-only caps (capture will likely fail to grab).";
}

// Parses the helper's -v log for its shmsink's negotiated input caps (format, width,
// height, framerate) and stores them in gpu_caps_fragment for the receiver-side
// capsfilter. Needed because shmsrc does not relay full caps over the shm protocol by
// itself in this GStreamer version — see the member's declaration comment in the header.
bool SpecificWorker::discover_gpu_caps()
{
    // Field order in GStreamer's serialized caps isn't guaranteed, so each field is
    // matched independently within the same line rather than as one rigid pattern.
    static const std::regex width_re(R"(width=\(int\)(\d+))");
    static const std::regex height_re(R"(height=\(int\)(\d+))");
    static const std::regex fps_re(R"(framerate=\(fraction\)(\d+/\d+))");

    for (int i = 0; i < 50; ++i) // up to 5s: camera negotiation/decoder warmup isn't instant
    {
        std::ifstream log("/tmp/ricoh_gpu_helper.log");
        std::string line;
        while (std::getline(log, line))
        {
            if (line.find("video/x-raw") != std::string::npos &&
                line.find("format=(string)BGR") != std::string::npos &&
                line.find("width=(int)") != std::string::npos)
            {
                std::smatch w, h, f;
                if (std::regex_search(line, w, width_re) && std::regex_search(line, h, height_re))
                {
                    gpu_caps_fragment = "video/x-raw,format=BGR,width=" + w[1].str() + ",height=" + h[1].str();
                    if (std::regex_search(line, f, fps_re))
                        gpu_caps_fragment += ",framerate=" + f[1].str();
                    std::cout << "GPU stream caps discovered: " << gpu_caps_fragment << std::endl;
                    return true;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

void SpecificWorker::emergency()
{
    fps.print("Emergency worker", 3000);
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}

//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
	//computeCODE
	//Restore emergency component

}

void SpecificWorker::self_adjust_period(int new_period)
{
    if(abs(new_period - getPeriod("Compute")) < 2)
        return;
    if(new_period > getPeriod("Compute"))
        {
            this->setPeriod("Compute", getPeriod("Compute") + 1);

        } else
        {
            this->setPeriod("Compute", getPeriod("Compute") - 1);

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

    try
    {
        last_read.store(std::chrono::high_resolution_clock::now());

        // Get image from double buffer
        auto img = buffer_image.get_idemp();
        if (img.empty())
            return res;

        // Bounds come from the actual retrieved frame, not from MAX_WIDTH/MAX_HEIGHT:
        // those two plain ints are updated (in compute()/reconnect_camera()) the instant
        // a new frame is grabbed, but buffer_image.put() swaps it in asynchronously on its
        // own worker thread, so right after a reconnect there is a window where they can
        // disagree with img's real size. Using img.cols/img.rows here makes the ROI math
        // safe regardless of that race, instead of risking an out-of-bounds cv::Rect.
        const int img_width = img.cols, img_height = img.rows;

        // Default values for invalid inputs
        sx = (sx <= 0) ? img_width : sx;
        sy = (sy <= 0) ? img_height : sy;
        cx = (cx < 0) ? img_width / 2 : cx;
        cy = (cy < 0) ? img_height / 2 : cy;
        roiwidth = (roiwidth < 0) ? img_width : roiwidth;
        roiheight = (roiheight < 0) ? img_height : roiheight;

        // Clamp requested ROI size to the actual frame, then adjust center to stay within bounds
        sx = std::clamp(sx, 1, img_width);
        sy = std::clamp(sy, 1, img_height);
        int half_sx = sx / 2, half_sy = sy / 2;
        cy = std::clamp(cy, half_sy, img_height - half_sy);
        cx = std::clamp(cx, half_sx, img_width - half_sx);

        cv::Mat roi;
        if (cx - half_sx < 0 || cx + half_sx > img_width) // Handle x out-of-bounds
        {
            cv::Mat left, right;
            if (cx - half_sx < 0)
            {
                img(cv::Rect(img_width - (half_sx - cx), cy - half_sy, half_sx - cx, sy)).copyTo(left);
                img(cv::Rect(0, cy - half_sy, cx + half_sx, sy)).copyTo(right);
            }
            else
            {
                img(cv::Rect(cx - half_sx, cy - half_sy, img_width - cx + half_sx, sy)).copyTo(left);
                img(cv::Rect(0, cy - half_sy, cx + half_sx - img_width, sy)).copyTo(right);
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
    }
    catch(const cv::Exception& e)
    {
        qWarning() << "Camera360RGB_getROI: OpenCV exception:" << e.what();
    }
    catch(const std::exception& e)
    {
        qWarning() << "Camera360RGB_getROI: exception:" << e.what();
    }

    return res;
}

std::string SpecificWorker::MediaPlaneDDS_getMediaDescriptor()
{
    // Report the live DDS 360 media-plane descriptor (JSON) so the robot_concept agent
    // can relay it into DSR. Empty when DDS publishing is disabled/failed, which tells
    // the consumer to fall back to bridging the panorama itself.
    if (dds_publisher)
        return dds_publisher->descriptor_json();
    return {};
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

