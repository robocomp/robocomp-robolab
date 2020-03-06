#include "vision.h"
#include "constants.h"

#include <stdio.h>

const int CAM_INFO_DEFAULT_URL_MAX_SIZE = 128;

Vision::Vision()
  : gst_pipeline_(NULL)
  , gst_sink_(NULL)
  , base_frame_id_(DEFAULT_BASE_FRAME_ID)
  , is_started_(false)
  , stop_requested_(false)
  , quit_requested_(false)
  , retry_count_(0)
  , camera_type_(CameraTypes::Unknown)
  , time_offset_(0)
  , image_width_(0)
  , image_height_(0)
  , pixel_depth_(0)
  , use_gst_timestamps_(false)
  , is_first_initialize_(true)
{
}

Vision::~Vision()
{
}

bool Vision::initialize()
{
  if (!gst_is_initialized())
  {
    gst_init(0, 0);
  }

  GError* error = 0;  // Assignment to zero is a gst requirement
  //camera_config_ =  "rtspsrc location=rtsp://192.168.1.10/color latency=30 ! rtph264depay ! avdec_h264 ! appsink name=sink ";
  camera_config_ = "rtspsrc location=rtsp://192.168.1.10/color latency=30 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink name=sink";
  gst_pipeline_ = gst_parse_launch(camera_config_.c_str(), &error);
  if (gst_pipeline_ == NULL)
  {
    std::cout << "gst_parse error " << camera_config_ << std::endl;
    return false;
  }
  
  camera_type_ = CameraTypes::Color;
  //  image_encoding_ = sensor_msgs::image_encodings::TYPE_16UC1; 
  
  // Create sink
  gst_sink_ = gst_bin_get_by_name (GST_BIN (gst_pipeline_), "sink");
  //gst_sink_ = gst_element_factory_make("sink", NULL);
  GstCaps* caps = gst_app_sink_get_caps(GST_APP_SINK(gst_sink_));
  
  // Set image encoding related to camera/stream type
  std::string gst_encoding = "RGB";
  pixel_depth_ = 3;  //bytes

  // if (image_encoding_ == sensor_msgs::image_encodings::TYPE_16UC1)
  // {
  //   gst_encoding = "GRAY16_LE";
  // }
 
  caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, gst_encoding.c_str(), NULL);

  gst_app_sink_set_caps(GST_APP_SINK(gst_sink_), caps);
  gst_caps_unref(caps);

  // if (GST_IS_PIPELINE(gst_pipeline_))
  // {
  //   GstPad* outpad = gst_bin_find_unlinked_pad(GST_BIN(gst_pipeline_), GST_PAD_SRC);
  //   g_assert(outpad);

  //   GstElement* outelement = gst_pad_get_parent_element(outpad);
  //   g_assert(outelement);
  //   gst_object_unref(outpad);

  //   if (!gst_bin_add(GST_BIN(gst_pipeline_), gst_sink_))
  //   {
  //     std::cout << "gst_bin_add failed " << camera_name_ << std::endl;
  //     gst_object_unref(outelement);
  //     gst_object_unref(gst_pipeline_);
  //     gst_pipeline_ = NULL;
  //     return false;
  //   }

  //   if (!gst_element_link(outelement, gst_sink_))
  //   {
  //     std::cout << "gstreamer: cannot link outelement " << gst_element_get_name(outelement) << " -> sink " << camera_name_ << std::endl;
  //     gst_object_unref(outelement);
  //     gst_object_unref(gst_pipeline_);
  //     gst_pipeline_ = NULL;
  //     return false;
  //   }

  //   gst_object_unref(outelement);
  // }
  // else
  // {
  //   GstElement* launchpipe = gst_pipeline_;
  //   gst_pipeline_ = gst_pipeline_new(NULL);
  //   g_assert(gst_pipeline_);

  //   gst_object_unparent(GST_OBJECT(launchpipe));

  //   gst_bin_add_many(GST_BIN(gst_pipeline_), launchpipe, gst_sink_, NULL);

  //   if (!gst_element_link(launchpipe, gst_sink_))
  //   {
  //     //ROS_FATAL("[%s]: gstreamer: cannot link launchpipe -> sink", camera_name_.c_str());
  //     gst_object_unref(gst_pipeline_);
  //     gst_pipeline_ = NULL;
  //     return false;
  //   }
  // }

  // Calibration between ros::Time and gst timestamps
  GstClock* clock = gst_system_clock_obtain();
  //ros::Time now = ros::Time::now();
  //GstClockTime ct = gst_clock_get_time(clock);
  gst_object_unref(clock);
  //time_offset_ = now.toSec() - GST_TIME_AS_USECONDS(ct) / 1e6;

  gst_element_set_state(gst_pipeline_, GST_STATE_PAUSED);

  if (gst_element_get_state(gst_pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE)
  {
    std::cout << "Failed to PAUSE stream, check your gstreamer configuration" << std::endl;
    return false;
  }
  else
  {
    std::cout << "Stream is PAUSED" << std::endl;
  }

  //loadCameraInfo();

  return true;
}

bool Vision::start()
{
  GstStateChangeReturn ret = gst_element_set_state(gst_pipeline_, GST_STATE_PLAYING);

  switch (ret)
  {
    case GST_STATE_CHANGE_FAILURE:
    case GST_STATE_CHANGE_NO_PREROLL:
      std::cout << "Failed to start stream" << camera_name_ << std::endl;
      return false;

    case GST_STATE_CHANGE_ASYNC:
    {
      ret = gst_element_get_state(gst_pipeline_, NULL, NULL, STATE_CHANGE_ASYNC_TIMEOUT * GST_SECOND);
      switch (ret)
      {
        case GST_STATE_CHANGE_FAILURE:
          std::cout << "Failed to start stream" << camera_name_ << std::endl;
          return false;

        case GST_STATE_CHANGE_ASYNC:
          std::cout << "Failed to start stream" << camera_name_ << std::endl;
          return false;
      }
    }
    case GST_STATE_CHANGE_SUCCESS:
      std::cout << "Stream started" << std::endl;
      break;

    default:
      return false;
  }

  // Get the frame width and height
  GstPad* pad = gst_element_get_static_pad(gst_sink_, "sink");
  const GstCaps* caps = gst_pad_get_current_caps(pad);

  GstStructure* structure = gst_caps_get_structure(caps, 0);
  gst_structure_get_int(structure, "width", &image_width_);
  gst_structure_get_int(structure, "height", &image_height_);

  std::cout << "width " << image_width_ << " height " << image_height_ << std::endl;

  gst_app_sink_set_max_buffers(GST_APP_SINK(gst_sink_), APP_SINK_BUFFER_COUNT);
  gst_app_sink_set_drop(GST_APP_SINK(gst_sink_), true);


  return true;
}

bool Vision::loadCameraInfo()
{
  std::string cam_info_default;
  char cam_info_default_resolved[CAM_INFO_DEFAULT_URL_MAX_SIZE];

  if (camera_info_.empty())
  {
    //snprintf(cam_info_default_resolved, CAM_INFO_DEFAULT_URL_MAX_SIZE, cam_info_default.c_str(), image_width_, image_height_);
    snprintf(cam_info_default_resolved, CAM_INFO_DEFAULT_URL_MAX_SIZE, "rtsp://192.168.1.10/color", 640, 480);
      
    camera_info_.assign(cam_info_default_resolved);
    std::cout << camera_info_ << " " << cam_info_default << " " << image_height_ << " " << image_width_ << std::endl;
  }

  return true;
}

std::tuple<bool, RoboCompCameraRGBDSimple::TImage> Vision::publish()
{
  GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(gst_sink_));
  if (!sample)
  {
    return std::make_tuple(false, RoboCompCameraRGBDSimple::TImage());
  }

  GstBuffer* buf = gst_sample_get_buffer(sample);
  if (!buf)
  {
    gst_sample_unref(sample);
    return std::make_tuple(false, RoboCompCameraRGBDSimple::TImage());
  }

  GstMapInfo map;
  if (!gst_buffer_map(buf, &map, GST_MAP_READ))
  {
    gst_sample_unref(sample);
    return std::make_tuple(false, RoboCompCameraRGBDSimple::TImage());
  }

  gsize& buf_size = map.size;
  guint8*& buf_data = map.data;

  //GstClockTime bt = gst_element_get_base_time(gst_pipeline_);

  unsigned int expected_frame_size = 0;
  expected_frame_size = image_width_ * image_height_ * pixel_depth_;
  //std::cout << "expected " << expected_frame_size << std::endl;

  // Complain if the returned buffer is smaller than we expect
  if (buf_size < expected_frame_size)
  {
    std::cout << " Image buffer underflow: expected frame to be  " << expected_frame_size << "bytes but got only " << buf_size << std::endl;
  }

  if (buf_size > expected_frame_size)
  {
    std::cout << " Image buffer overflow: expected frame to be  " << expected_frame_size << "bytes but got only " << buf_size << std::endl;
    
    gst_buffer_unmap(buf, &map);
    gst_sample_unref(sample);

    return std::make_tuple(false, RoboCompCameraRGBDSimple::TImage());
  }

  // Image data and metadata
  RoboCompCameraRGBDSimple::TImage img;
  img.width = image_width_;
  img.height = image_height_;
  img.depth = pixel_depth_;
  //img.encoding = image_encoding_;
  //img.is_bigendian = false;
  img.image.resize(expected_frame_size);
  //img.step = image_width_ * pixel_size_;

  // Copy only the data we received
  // Since we're publishing shared pointers, we need to copy the image so
  // we can free the buffer allocated by gstreamer
  std::copy(buf_data, (buf_data) + (buf_size), img.image.begin());

  gst_buffer_unmap(buf, &map);
  gst_sample_unref(sample);

  return std::make_tuple(true, img);
}

void Vision::stop()
{
  stop_requested_ = true;
  is_started_ = false;

  if (gst_pipeline_)
  {
    changePipelineState(GST_STATE_PAUSED);
    changePipelineState(GST_STATE_READY);
    changePipelineState(GST_STATE_NULL);

    gst_object_unref(gst_pipeline_);
    gst_pipeline_ = NULL;
  }
}

bool Vision::changePipelineState(GstState state)
{
  GstStateChangeReturn ret;

  ret = gst_element_set_state(gst_pipeline_, state);
  if (GST_STATE_CHANGE_ASYNC == ret)
  {
    ret = gst_element_get_state(gst_pipeline_, NULL, NULL, STATE_CHANGE_ASYNC_TIMEOUT * GST_SECOND);
  }

  switch (ret)
  {
    case GST_STATE_CHANGE_SUCCESS:
    case GST_STATE_CHANGE_NO_PREROLL:
      return true;

    case GST_STATE_CHANGE_FAILURE:
      //ROS_ERROR("[%s]: Failed to change pipeline state to %s",
      //          camera_name_.c_str(), gst_element_state_get_name(state));
      return false;

    case GST_STATE_CHANGE_ASYNC:
    {
      //ROS_ERROR("[%s]: Failed to change pipeline state to %s (timeout)",
      //          camera_name_.c_str(), gst_element_state_get_name(state));
      return false;
    }

    default:
      //ROS_ERROR("[%s]: Unknown state change return value when trying to change pipeline state to %s",
       //         camera_name_.c_str(), gst_element_state_get_name(state));
      return false;
  }
}

void Vision::quit()
{
  quit_requested_ = true;

  if (gst_pipeline_)
  {
    GstEvent *event = NULL;
    event = gst_event_new_eos();
    gst_element_send_event(gst_pipeline_, event);
  }
}