// Copyright 2022 Jonathan Bohren, Clyde McQueen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdlib.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <iostream>
#include <string>

extern "C" {
#include "gst/gst.h"
#include "gst/app/gstappsink.h"
}

#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "RosGstCamPublishStream.hpp"

namespace campublishstream
{

CamPublishStream::CamPublishStream(const rclcpp::NodeOptions & options, GstCamConfig *camcfg)
: rclcpp::Node("Ros2CamPublisherNode", options),
  gsconfig_(""),
  pipeline_(NULL),
  sink_(NULL),
  srcelement_(NULL),
  isQmmfsrc(true),
  camera_info_manager_(this),
  stop_signal_(false)
{
  p_CamCfg = camcfg->GetRos2CameraConfig();
  
  pipeline_thread_ = std::thread(
    [this]()
    {
      run();
    });
}

CamPublishStream::~CamPublishStream()
{
  stop_signal_ = true;
  pipeline_thread_.join();
}

bool CamPublishStream::configure()
{
  // Get gstreamer configuration
  // (either from environment variable or ROS param)
  std::stringstream ss;
  bool gsconfig_rosparam_defined = false;
  const auto gsconfig_rosparam = declare_parameter("gscam_config", "");
  gsconfig_rosparam_defined = !gsconfig_rosparam.empty();

  if(p_CamCfg->cameraSource == "qtiqmmfsrc" )
    isQmmfsrc = true;
  else
    isQmmfsrc = false;

  if(isQmmfsrc)
    ss << "qtiqmmfsrc name=qmmf af-mode=3" << " ! " << "video/x-raw,framerate="<< p_CamCfg->fps << ",format=" << p_CamCfg->format.c_str() << ",width="
        << p_CamCfg->mWidth << ",height=" << p_CamCfg->mHeight << " ! " << "videoconvert";
  else
    ss << "v4l2src device=/dev/video" << p_CamCfg->cameraId << " ! " << "video/x-raw,framerate="<< p_CamCfg->fps << ",format=" << p_CamCfg->format.c_str() 
    << ",width=" << p_CamCfg->mWidth << ",height=" << p_CamCfg->mHeight << " ! " << "videoconvert";
    
  if(gsconfig_rosparam_defined)
    gsconfig_ = gsconfig_rosparam;
  else
    gsconfig_ = ss.str();
  RCLCPP_INFO(get_logger(), "CamPublishStream: %s\n", gsconfig_.c_str());
  
  // Get additional gscam configuration
  sync_sink_ = declare_parameter("sync_sink", true);
  preroll_ = declare_parameter("preroll", false);
  use_gst_timestamps_ = declare_parameter("use_gst_timestamps", false);

  reopen_on_eof_ = declare_parameter("reopen_on_eof", false);

  // Get the camera parameters file
  camera_info_url_ = declare_parameter("camera_info_url", "");
  camera_name_ = declare_parameter("camera_name", "default");

  // Get the image encoding
  image_encoding_ =
    declare_parameter("image_encoding", std::string(sensor_msgs::image_encodings::RGB8));
  if(image_encoding_.empty())
    image_encoding_ = p_CamCfg->image_encoding;
  if (image_encoding_ != sensor_msgs::image_encodings::RGB8 &&
    image_encoding_ != sensor_msgs::image_encodings::MONO8 &&
    image_encoding_ != sensor_msgs::image_encodings::YUV422 &&
    image_encoding_ != "jpeg")
  {
    RCLCPP_FATAL_STREAM(get_logger(), "Unsupported image encoding: " + image_encoding_);
  }

  camera_info_manager_.setCameraName(camera_name_);

  if (camera_info_manager_.validateURL(camera_info_url_)) {
    camera_info_manager_.loadCameraInfo(camera_info_url_);
    RCLCPP_INFO_STREAM(get_logger(), "Loaded camera calibration from " << camera_info_url_);
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Camera info at: " << camera_info_url_ << " not found. Using an uncalibrated config.");
  }

  // Get TF Frame
  frame_id_ = declare_parameter("frame_id", "camera_frame");
  if (frame_id_ == "camera_frame") {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "No camera frame_id set, using frame \"" << frame_id_ << "\".");
  }

  use_sensor_data_qos_ = declare_parameter("use_sensor_data_qos", false);

  return true;
}

bool CamPublishStream::init_stream()
{
  RCLCPP_DEBUG_STREAM(get_logger(), "Gstreamer Version: " << gst_version_string() );

  GError * error = 0;  // Assignment to zero is a gst requirement

  pipeline_ = gst_parse_launch(gsconfig_.c_str(), &error);
  if (pipeline_ == NULL) {
    RCLCPP_FATAL_STREAM(get_logger(), error->message);
    return false;
  }
  if(isQmmfsrc)
    srcelement_ = gst_bin_get_by_name(GST_BIN(pipeline_), "qmmf");
  else
    srcelement_ = gst_bin_get_by_name(GST_BIN(pipeline_), "v4l2dev");

  // Create RGB sink
  sink_ = gst_element_factory_make("appsink", NULL);
  GstCaps * caps = gst_app_sink_get_caps(GST_APP_SINK(sink_));

  // http://gstreamer.freedesktop.org/data/doc/gstreamer/head/pwg/html/section-types-definitions.html
  if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
    caps = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "RGB",
      NULL);
  } else if (image_encoding_ == sensor_msgs::image_encodings::MONO8) {
    caps = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "GRAY8",
      NULL);
  } else if (image_encoding_ == sensor_msgs::image_encodings::YUV422) {
    caps = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "UYVY",
      NULL);
  } else if (image_encoding_ == "jpeg") {
    caps = gst_caps_new_simple("image/jpeg", NULL, NULL);
  }

  gst_app_sink_set_caps(GST_APP_SINK(sink_), caps);
  gst_caps_unref(caps);

  // Set whether the sink should sync
  // Sometimes setting this to true can cause a large number of frames to be
  // dropped
  gst_base_sink_set_sync(
    GST_BASE_SINK(sink_),
    (sync_sink_) ? TRUE : FALSE);

  if (GST_IS_PIPELINE(pipeline_)) {
    GstPad * outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline_), GST_PAD_SRC);
    g_assert(outpad);

    GstElement * outelement = gst_pad_get_parent_element(outpad);
    g_assert(outelement);
    gst_object_unref(outpad);

    if (!gst_bin_add(GST_BIN(pipeline_), sink_)) {
      RCLCPP_FATAL(get_logger(), "gst_bin_add() failed");
      gst_object_unref(outelement);
      gst_object_unref(pipeline_);
      return false;
    }

    if (!gst_element_link(outelement, sink_)) {
      RCLCPP_FATAL(
        get_logger(), "GStreamer: cannot link outelement(\"%s\") -> sink\n",
        gst_element_get_name(outelement));
      gst_object_unref(outelement);
      gst_object_unref(pipeline_);
      return false;
    }

    gst_object_unref(outelement);
  } else {
    GstElement * launchpipe = pipeline_;
    pipeline_ = gst_pipeline_new(NULL);
    g_assert(pipeline_);

    gst_object_unparent(GST_OBJECT(launchpipe));

    gst_bin_add_many(GST_BIN(pipeline_), launchpipe, sink_, NULL);

    if (!gst_element_link(launchpipe, sink_)) {
      RCLCPP_FATAL(get_logger(), "GStreamer: cannot link launchpipe -> sink");
      gst_object_unref(pipeline_);
      return false;
    }
  }

  // Calibration between ros::Time and gst timestamps
  GstClock * clock = gst_system_clock_obtain();
  GstClockTime ct = gst_clock_get_time(clock);
  gst_object_unref(clock);
  time_offset_ = now().nanoseconds() - GST_TIME_AS_NSECONDS(ct);
  RCLCPP_INFO(get_logger(), "Time offset: %.6f", rclcpp::Time(time_offset_).seconds());

  gst_element_set_state(pipeline_, GST_STATE_PAUSED);

  if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_FATAL(get_logger(), "Failed to PAUSE stream, check your gstreamer configuration.");
    return false;
  } else {
    RCLCPP_DEBUG_STREAM(get_logger(), "Stream is PAUSED.");
  }

  // Create ROS camera interface
  const auto qos = use_sensor_data_qos_ ? rclcpp::SensorDataQoS() : rclcpp::QoS{1};
  if (image_encoding_ == "jpeg") {
    jpeg_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>(
      "camera/image_raw/compressed", qos);
    cinfo_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera/camera_info", qos);
  } else {
    camera_pub_ = image_transport::create_camera_publisher(
      this, "camera/image_raw", qos.get_rmw_qos_profile());
  }

  return true;
}

void CamPublishStream::publish_stream()
{
  RCLCPP_INFO_STREAM(get_logger(), "Publishing stream...");

  // Pre-roll camera if needed
  if (preroll_) {
    RCLCPP_DEBUG(get_logger(), "Performing preroll...");

    // The PAUSE, PLAY, PAUSE, PLAY cycle is to ensure proper pre-roll
    // I am told this is needed and am erring on the side of caution.
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to PLAY during preroll.");
      return;
    } else {
      RCLCPP_DEBUG(get_logger(), "Stream is PLAYING in preroll.");
    }

    gst_element_set_state(pipeline_, GST_STATE_PAUSED);
    if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to PAUSE.");
      return;
    } else {
      RCLCPP_INFO(get_logger(), "Stream is PAUSED in preroll.");
    }
  }

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(get_logger(), "Could not start stream!");
    return;
  }
  RCLCPP_INFO(get_logger(), "Started stream.");

  // Poll the data as fast a spossible
  while (!stop_signal_ && rclcpp::ok()) {
    // This should block until a new frame is awake, this way, we'll run at the
    // actual capture framerate of the device.
    // RCLCPP_DEBUG(get_logger(), "Getting data...");
    GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK(sink_));
    if (!sample) {
      RCLCPP_ERROR(get_logger(), "Could not get gstreamer sample.");
      break;
    }
    GstBuffer * buf = gst_sample_get_buffer(sample);
    GstMemory * memory = gst_buffer_get_memory(buf, 0);
    GstMapInfo info;

    gst_memory_map(memory, &info, GST_MAP_READ);
    gsize & buf_size = info.size;
    guint8 * & buf_data = info.data;

    GstClockTime bt = gst_element_get_base_time(pipeline_);

    // Stop on end of stream
    if (!buf) {
      RCLCPP_INFO(get_logger(), "Stream ended.");
      break;
    }

    // RCLCPP_DEBUG(get_logger(), "Got data.");

    // Get the image width and height
    GstPad * pad = gst_element_get_static_pad(sink_, "sink");
    const GstCaps * caps = gst_pad_get_current_caps(pad);
    GstStructure * structure = gst_caps_get_structure(caps, 0);
    gst_structure_get_int(structure, "width", &width_);
    gst_structure_get_int(structure, "height", &height_);

    // Update header information
    sensor_msgs::msg::CameraInfo cur_cinfo = camera_info_manager_.getCameraInfo();
    sensor_msgs::msg::CameraInfo::SharedPtr cinfo;
    cinfo.reset(new sensor_msgs::msg::CameraInfo(cur_cinfo));
    if (use_gst_timestamps_) {
      cinfo->header.stamp = rclcpp::Time(GST_TIME_AS_NSECONDS(buf->pts + bt) + time_offset_);
    } else {
      cinfo->header.stamp = now();
    }
    // RCLCPP_INFO(get_logger(), "Image time stamp: %.3f",cinfo->header.stamp.toSec());
    cinfo->header.frame_id = frame_id_;
    if (image_encoding_ == "jpeg") {
      sensor_msgs::msg::CompressedImage::SharedPtr img(new sensor_msgs::msg::CompressedImage());
      img->header = cinfo->header;
      img->format = "jpeg";
      img->data.resize(buf_size);
      std::copy(
        buf_data, (buf_data) + (buf_size),
        img->data.begin());
      jpeg_pub_->publish(*img);
      cinfo_pub_->publish(*cinfo);
    } else {
      // Complain if the returned buffer is smaller than we expect
      const unsigned int expected_frame_size =
        width_ * height_ * sensor_msgs::image_encodings::numChannels(image_encoding_);

      if (buf_size < expected_frame_size) {
        RCLCPP_WARN_STREAM(
          get_logger(), "GStreamer image buffer underflow: Expected frame to be " <<
            expected_frame_size << " bytes but got only " <<
            buf_size << " bytes. (make sure frames are correctly encoded)");
      }

      // Construct Image message
      sensor_msgs::msg::Image::SharedPtr img(new sensor_msgs::msg::Image());

      img->header = cinfo->header;

      // Image data and metadata
      img->width = width_;
      img->height = height_;
      img->encoding = image_encoding_;
      img->is_bigendian = false;
      img->data.resize(expected_frame_size);

      // Copy only the data we received
      // Since we're publishing shared pointers, we need to copy the image so
      // we can free the buffer allocated by gstreamer
      img->step = width_ * sensor_msgs::image_encodings::numChannels(image_encoding_);

      std::copy(
        buf_data,
        (buf_data) + (buf_size),
        img->data.begin());

      // Publish the image/info
      camera_pub_.publish(img, cinfo);
    }

    // Release the buffer
    if (buf) {
      gst_memory_unmap(memory, &info);
      gst_memory_unref(memory);
      gst_sample_unref(sample);
    }
  }
}

void CamPublishStream::cleanup_stream()
{
  // Clean up
  RCLCPP_INFO(get_logger(), "Stopping gstreamer pipeline...");
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = NULL;
  }
}

void CamPublishStream::run()
{
  if (!this->configure()) {
    RCLCPP_FATAL(get_logger(), "Failed to configure gscam!");
    return;
  }

  while (!stop_signal_ && rclcpp::ok()) {
    if (!this->init_stream()) {
      RCLCPP_FATAL(get_logger(), "Failed to initialize gscam stream!");
      break;
    }

    // Block while publishing
    this->publish_stream();

    this->cleanup_stream();

    RCLCPP_INFO(get_logger(), "GStreamer stream stopped!");

    if (reopen_on_eof_) {
      RCLCPP_INFO(get_logger(), "Reopening stream...");
    } else {
      RCLCPP_INFO(get_logger(), "Cleaning up stream and exiting...");
      break;
    }
  }
  //rclcpp::shutdown();
}

int32_t CamPublishStream::setZoom(uint32_t size)
{
    if(!isQmmfsrc)
    {
      RCLCPP_INFO(get_logger() , "v4l2 camera  not support zoom! \n");
      return 0;
    }
    RCLCPP_INFO(get_logger() , " setzoom start \n");
    gchar str[64];
    gint location_x;
    gint location_y;
    gint w;
    gint h;
    GValue value = G_VALUE_INIT;
  
    if( size < 1 ||  size > 8 ) {
        RCLCPP_INFO(get_logger(), "zoom unsupport size less than 1 and more than 8,  %d\n", size);
        return 1;
    }
  
    w = p_CamCfg->rWidth / size;
    h = p_CamCfg->rHeight / size;
  
    if( size == 1)
    {
        location_x = 0;
        location_y = 0;
    } else {
    //center zoom mode
        location_x = (p_CamCfg->rWidth - w) / 2;
        location_y = (p_CamCfg->rHeight - h) / 2;
    }

    snprintf (str, sizeof(str), "<%d, %d, %d, %d>", location_x, location_y, w , h);
    RCLCPP_INFO(get_logger(), "zoom para: %s \n",str);
    g_value_init (&value, GST_TYPE_ARRAY);
    if (gst_value_deserialize (&value, str)) {
        g_object_set_property (G_OBJECT (srcelement_), "zoom", &value);
    }
 
    return 0;
}

// Example callbacks for appsink
// TODO(someone): enable callback-based capture
void gst_eos_cb(GstAppSink * appsink, gpointer user_data)
{
}
GstFlowReturn gst_new_preroll_cb(GstAppSink * appsink, gpointer user_data)
{
  return GST_FLOW_OK;
}
GstFlowReturn gst_new_asample_cb(GstAppSink * appsink, gpointer user_data)
{
  return GST_FLOW_OK;
}

}  // namespace campublishstream

//#include "rclcpp_components/register_node_macro.hpp"

//RCLCPP_COMPONENTS_REGISTER_NODE(campublishstream::CamPublishStream)