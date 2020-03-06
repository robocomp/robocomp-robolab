#ifndef KINOVA_VISION_H
#define KINOVA_VISION_H

#include <string>
#include <CameraRGBDSimpleYoloPub.h>

extern "C" {
#include <gstreamer-1.0/gst/gst.h>
#include <gst/app/gstappsink.h>
}

namespace CameraTypes
{
  enum CameraType
  {
    Unknown = 0,
    Color = 1,
    Depth = 2,
  };
}

class Vision
{
  public:
    Vision();
    ~Vision();
    bool start();
    void stop();
    bool initialize();
    void quit();
    std::tuple<bool, RoboCompCameraRGBDSimple::TImage> publish();
    
  private:
    bool changePipelineState(GstState state);
    bool loadCameraInfo();

    // Gstreamer elements
    GstElement* gst_pipeline_;
    GstElement* gst_sink_;

    // General gstreamer configuration
    std::string camera_config_;
    std::string camera_name_;
    std::string camera_info_;
    std::string frame_id_;
    std::string image_encoding_;
    std::string base_frame_id_;

    bool is_started_;
    bool stop_requested_;
    bool quit_requested_;
    int retry_count_;
    int camera_type_;
    double time_offset_;
    int image_width_;
    int image_height_;
    int pixel_depth_;
    bool use_gst_timestamps_;
    bool is_first_initialize_;


};

#endif
