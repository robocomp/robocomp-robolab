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

/*
 * # RicohOmni

Installation of Ricoh drivers

Goto: https://codetricity.github.io/theta-linux/software/  and choose the

    Using gstreamer and OpenCV without v4l2loopback

option, and install https://github.com/nickel110/gstthetauvc

Install de NVidia GStreamer plugin with: https://gist.github.com/corenel/a615b6f7eb5b5425aa49343a7b409200

Add to .bashrc and source:
    export GST_PLUGIN_PATH=/home/robocomp/software/gstthetauvc/thetauvc

 */

/**
	\brief
	@author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

// HIBERNATION DESACTIVADA para mantener máxima velocidad continua
// Si se activa, reduce el período cuando no hay peticiones
// #define HIBERNATION_ENABLED

#include <genericworker.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <fps/fps.h>
#include <doublebuffer/DoubleBuffer.h>
#include <QProcess>
#include <thread>
#include <chrono>
#include <memory>

using namespace std::chrono;

// Zero-copy DDS media-plane publisher (Image360), gated by config. FastDDS headers
// stay behind this forward declaration (PIMPL in dds_publisher.cpp).
class Ricoh360DDSPublisher;

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    /**
     * \brief Constructor for SpecificWorker.
     * \param configLoader Configuration loader for the component.
     * \param tprx Tuple of proxies required for the component.
     * \param startup_check Indicates whether to perform startup checks.
     */
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);

	/**
     * \brief Destructor for SpecificWorker.
     */
	~SpecificWorker();

	RoboCompCamera360RGB::TImage Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight);

	std::string MediaPlaneDDS_getMediaDescriptor();

public slots:

	/**
	 * \brief Initializes the worker one time.
	 */
	void initialize();

	/**
	 * \brief Main compute loop of the worker.
	 */
	void compute();

	/**
	 * \brief Handles the emergency state loop.
	 */
	void emergency();

	/**
	 * \brief Restores the component from an emergency state.
	 */
	void restore();

    /**
     * \brief Performs startup checks for the component.
     * \return An integer representing the result of the checks.
     */
	int startup_check();

private:

	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;
	bool activated_camera = false;
	cv::Mat cv_frame;
	cv::VideoCapture capture;

	// Streaming paralelo a MediaMTX
	cv::VideoWriter stream_writer;
	std::string stream_pipeline;
	bool streaming_enabled = false;

	std::vector<int> compression_params;
	std::string pipeline;

	FPSCounter fps;
	std::atomic<std::chrono::high_resolution_clock::time_point> last_read;
	int MAX_INACTIVE_TIME = 5;  // secs after which the component is paused. It reactivates with a new reset

	struct PARAMS
	{
		std::string device = "/dev/video0";
		bool display = false;
		bool orin = false;
		bool use_gpu = true;
		std::string gpu_decoder = "nvh264dec"; // GStreamer NVDEC element name; varies by nvcodec plugin build ("nvh264dec" vs older "nvdec")
		std::string gpu_downloader = "gldownload"; // "gldownload" (needs a real GL/DRI2 display) or "cudadownload" (no display needed)
		// GPU decode runs in an isolated child process (see launch_gpu_helper()): nvh264dec
		// needs h264parse, which carries a reproducible upstream segfault
		// (GstBaseParse::gst_base_parse_set_latency — see debug_ricoh_omni_dds.md). Frames
		// cross into this process over a GStreamer shmsink/shmsrc channel.
		std::string gpu_shm_socket = "/tmp/ricoh_omni_dds_gpu.sock";
		int gpu_shm_size = 100000000; // ~100MB ring buffer, a few 2K BGR frames' worth
		// avdec_h264/x264enc default to 0 ("auto" = one worker thread per detected CPU
		// core). On a high core-count machine that alone can push CPU into the hundreds
		// of percent even with a single stream — cap them explicitly per machine.
		int decode_threads = 4;
		int encode_threads = 2;
		bool compressed = false;
		bool simulator = false;
		bool streaming = true;      // Streaming paralelo a MediaMTX
		bool stream_webrtc = false; // true=WHIP/WebRTC directo, false=RTMP a MediaMTX
		std::string stream_host = "localhost";
		int stream_port = 8554;
		std::string stream_path = "theta";
		int time_offset;

		// MediaMTX gestionado por el propio componente (solo si !simulator)
		bool launch_mediamtx = true;
		std::string mediamtx_bin = "./mediamtx";
		std::string mediamtx_config = "mediamtx.yml";
	};
	PARAMS pars;
	int MAX_WIDTH, MAX_HEIGHT, DEPTH;
	RoboCompCamera360RGB::TImage image;

	DoubleBuffer<cv::Mat, cv::Mat> buffer_image;

	long long capture_time;
	// ★ Dedup key for the SIMULATED source. getROI is a PULL: asking faster than the source produces
	// returns the same panorama again, and without this the component republished it — measured
	// 2026-09-10 on the live plane, 16% of frames carried a stamp already sent. That is not just
	// wasted bandwidth: a consumer re-runs its perception on data it already processed, and any
	// latency measured from the stamp reads high because the repeat is timed at a later wall clock.
	// The real-camera branch needs no key — capture.read() only returns on a genuine new frame.
	long long last_source_timestamp_ = -1;
	// Source period as a DECAYING MINIMUM, and the poll locked to it. A missed frame can only make a
	// stamp delta an integer MULTIPLE of the true period, never smaller, so an average would inflate
	// under drops and slow the poll, which drops more — a minimum cannot.
	// ★ The 4x factor is not an oversample any more: getROI blocks at the bridge until the next
	// panorama, so an early poll transfers nothing, it waits. What the factor buys is being ALREADY
	// BLOCKED when the frame lands — a consuming get() returns a frame that is merely sitting in the
	// buffer immediately, aged by however long it sat, so asking slower than the source restores the
	// polling phase we removed. It only has to stay comfortably below the source period, and the CPU
	// cost that used to bound it is gone with the duplicate transfers. Plane lag 50 -> 8.0 ms.
	double src_period_ms_ = -1.0;
	// track image period
	void self_adjust_period(int new_period);

	// Optional zero-copy DDS 360 media plane (null unless PublishDDS is enabled).
	bool publish_dds = false;
	std::unique_ptr<Ricoh360DDSPublisher> dds_publisher;

	// MediaMTX child process (real hardware mode only). Launched detached (not as a
	// QProcess instance) because the connection-retry loop below blocks the Qt event
	// loop, and a still-running QProcess instance gets killed when it is destroyed
	// without the event loop ever having pumped its child-exit notification.
	qint64 mediamtx_pid = -1;
	void launch_mediamtx();

	// GPU decode helper child process (only when pars.use_gpu). Same launch pattern as
	// mediamtx: detached via a shell (not a QProcess instance) so a blocking retry loop
	// elsewhere doesn't get it killed on destruction before the event loop ever pumps
	// its child-exit notification.
	qint64 gpu_helper_pid = -1;
	std::string gpu_helper_pipeline;
	void launch_gpu_helper();
	bool gpu_helper_alive() const;

	// shmsrc doesn't relay the sender's negotiated caps automatically (confirmed: with
	// no capsfilter, appsink samples have no caps at all; with a format-only capsfilter,
	// OpenCV's gst_video_info_from_caps() fails — "can't handle unknown layout" — width
	// and height are required). Since the real resolution depends on what the camera/
	// decoder negotiate and isn't fixed in config, it's discovered at runtime by parsing
	// the helper's own verbose (-v) log for its shmsink's negotiated input caps, and
	// used to build a fully-specified receiver-side capsfilter.
	std::string gpu_caps_fragment = "video/x-raw,format=BGR";
	bool discover_gpu_caps();

	bool reconnect_camera();
	void maybe_reconnect();
	std::chrono::steady_clock::time_point last_reconnect;

	// cv::VideoWriter::write() has no way to report a push failure to application code
	// on the GStreamer backend (writeFrame() only logs a warning internally — no
	// exception, no return value) — a stalled RTMP link that mediamtx eventually drops
	// (logged there as "i/o timeout"/"EOF") is therefore invisible to us: our catch
	// block never fires, streaming_enabled never flips false, and nothing ever reopens
	// it. Confirmed live on shadow-hp 2026-08-13 (mediamtx.log showed the disconnect,
	// this process kept calling write() into a dead pipe indefinitely).
	//
	// First mitigation (2026-08-13) was an unconditional periodic refresh every 120s —
	// worked, but tore down a perfectly healthy connection every single cycle, causing
	// a visible ~120s-periodic glitch in the remote view (confirmed live on
	// robolab-shadow 2026-09-24 via mediamtx.log: publish/EOF/publish every exactly
	// 120s). Replaced with an actual health check against mediamtx's own HTTP API
	// (requires `api: yes` in mediamtx.yml — see mediamtx_path_has_publisher()): poll
	// cheaply and often, but only reopen stream_writer when mediamtx itself reports no
	// active publisher for our path, instead of on a blind timer.
	std::chrono::steady_clock::time_point last_stream_refresh;
	static constexpr int STREAM_HEALTH_CHECK_INTERVAL_S = 5;
	bool mediamtx_path_has_publisher() const;

	// Adaptive backoff: doubles on each failed reconnect (capped), resets to base on
	// success. Without this, a flapping USB link retries every fixed 2s forever, each
	// attempt potentially leaking a GStreamer pipeline that release() couldn't fully
	// tear down (source/sink stuck mid-teardown) — backing off slows the leak rate.
	int reconnect_cooldown_s = 2;
	static constexpr int MAX_RECONNECT_COOLDOWN_S = 30;

	// Was a function-local `static int` in compute(): that made it a one-shot,
	// process-lifetime counter, so once tripped it re-tripped on the very first write
	// after every future reopen, causing open/immediate-close churn on stream_writer.
	// As a member it's reset per streaming session (see reconnect_camera()/compute()).
	int stream_write_errors = 0;

signals:
	//void customSignal();
};

#endif
