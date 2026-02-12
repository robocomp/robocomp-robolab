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
/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}
void SpecificWorker::initialize()
{
try
    {
        //Parametros por el config
        pars.display = this->configLoader.get<bool>("display") or (this->configLoader.get<bool>("display"));
        pars.simulator = this->configLoader.get<bool>("simulator") or (this->configLoader.get<bool>("simulator"));
        pars.orin = this->configLoader.get<bool>("orin") or (this->configLoader.get<bool>("orin"));
        pars.compressed = this->configLoader.get<bool>("compressed") or (this->configLoader.get<bool>("compressed"));
        pars.time_offset =this->configLoader.get<double>("time_offset");

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

        std::cout << "Params: device" << pars.device << " display " << pars.display << " compressed: " << pars.compressed
                  << " simulator: " << pars.simulator << " streaming: " << pars.streaming << " offset "<< pars.time_offset <<std::endl;
    }
    catch(const std::exception &e)
    { std::cout << e.what() << " Error reading config params" << std::endl;};

//chekpoint robocompUpdater
    std::cout << "Initializing worker" << std::endl;

	if(this->startup_check_flag)
		this->startup_check();
	else
    {
        last_read.store(std::chrono::high_resolution_clock::now());

        // Pipeline optimizado para máxima velocidad y FPS
        // - Sin queues intermedias (reduce latencia)
        // - max-buffers=1 en appsink (solo último frame)
        // - drop=true (descarta frames antiguos)
        // - sync=false (no sincronización, máxima velocidad)
        if (pars.orin)
            pipeline = "thetauvcsrc mode=2K ! h264parse ! nvv4l2decoder ! nvvidconv ! videoconvert n-threads=4 ! video/x-raw,format=BGR ! appsink max-buffers=1 drop=true sync=false";
        else
            pipeline = "thetauvcsrc mode=2K ! h264parse ! nvh264sldec ! videoconvert n-threads=4 ! video/x-raw,format=BGR ! appsink max-buffers=1 drop=true sync=false";
        //if (pars.orin)
        //pipeline = "thetauvcsrc mode=2K ! h264parse ! nvv4l2decoder ! nvvidconv ! videoconvert n-threads=2 ! video/x-raw,format=BGR ! appsink drop=true sync=false";
        //else
        //pipeline = "thetauvcsrc mode=2K ! h264parse ! nvdec ! gldownload ! videoconvert n-threads=2 ! video/x-raw,format=BGR ! appsink drop=true sync=false";

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
            // Usamos shmsink para compartir frames con un proceso GStreamer externo
            // O usamos appsrc -> flvmux -> rtmpsink (RTMP es más compatible)
            stream_pipeline = "appsrc ! videoconvert ! video/x-raw,format=I420 ! "
                            "x264enc tune=zerolatency bitrate=5000 speed-preset=ultrafast key-int-max=30 ! "
                            "video/x-h264,profile=baseline ! "
                            "flvmux streamable=true ! "
                            "rtmpsink location=rtmp://" + pars.stream_host + ":1935/" + pars.stream_path + " sync=false";

            std::cout << "Stream pipeline: " << stream_pipeline << std::endl;

            // Intentar abrir el writer (sin bloquear si falla)
            try {
                if(stream_writer.open(stream_pipeline, cv::CAP_GSTREAMER, 0, 30,
                                     cv::Size(MAX_WIDTH, MAX_HEIGHT), true))
                {
                    streaming_enabled = true;
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
    // OPTIMIZADO PARA MÁXIMA VELOCIDAD - Sin pausas, captura continua

    bool frame_valid = false;

    if(not pars.simulator)
    {
        // Real camera capture - optimizado para velocidad
        try
        {
            if(capture.isOpened())
            {
                // Usar read() directo es más rápido que grab()+retrieve()
                capture.read(cv_frame);

                // Validación mínima (solo empty check para máxima velocidad)
                if(!cv_frame.empty())
                {
                    capture_time = duration_cast<milliseconds>(
                        system_clock::now().time_since_epoch()).count() - pars.time_offset;
                    frame_valid = true;
                }
            }
            else
            {
                qWarning() << "Camera not opened";
                activated_camera = false;
            }
        }
        catch(const cv::Exception& e)
        {
            qWarning() << "OpenCV exception:" << e.what();
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
        // Publicar frame al stream de MediaMTX (en paralelo, sin bloquear)
        if(streaming_enabled && stream_writer.isOpened())
        {
            try {
                // Escribir frame sin bloquear el loop principal
                stream_writer.write(cv_frame);
            }
            catch(const cv::Exception& e) {
                // Silenciar errores de streaming para no ralentizar captura
                // Si falla consistentemente, deshabilitamos streaming
                static int stream_errors = 0;
                if(++stream_errors > 100) {
                    std::cout << "⚠ Too many streaming errors, disabling..." << std::endl;
                    streaming_enabled = false;
                    stream_writer.release();
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

