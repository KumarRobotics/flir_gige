#include "thermal_proc/thermal_proc_node.h"

#include <algorithm>
#include <cmath>

#include <opencv2/contrib/contrib.hpp>

namespace flir_gige {

ThermalProcNode::ThermalProcNode(const ros::NodeHandle &nh,
                                 const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), it_(nh), cfg_server_(pnh) {
  image_transport::SubscriberStatusCallback connect_cb =
      boost::bind(&ThermalProcNode::ConnectCb, this);
  pub_proc_ = it_.advertise("image_proc", 1, connect_cb, connect_cb);

  cfg_server_.setCallback(
      boost::bind(&ThermalProcNode::ConfigCb, this, _1, _2));
}

void ThermalProcNode::ConnectCb() {
  std::lock_guard<std::mutex> lock(connect_mutex_);
  if (!pub_proc_.getNumSubscribers())
    sub_camera_.shutdown();
  else if (!sub_camera_) {
    image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);
    sub_camera_ = it_.subscribeCamera("image_raw", 2,
                                      &ThermalProcNode::CameraCb, this, hints);
  }
}

void ThermalProcNode::ConfigCb(ThermalProcDynConfig &config, int level) {
  if (level < 0) {
    ROS_INFO("%s: %s", pnh_.getNamespace().c_str(),
             "Initializaing reconfigure server");
  }
  // Make sure that max is greater than min
  config.celsius_max = (config.celsius_max > config.celsius_min)
                           ? config.celsius_max
                           : (config.celsius_min + 5);
  config_ = config;
}

void ThermalProcNode::CameraCb(
    const sensor_msgs::ImageConstPtr &image_msg,
    const sensor_msgs::CameraInfoConstPtr &cinfo_msg) {
  // Verify camera is actually calibrated
  if (cinfo_msg->K[0] == 0.0 || cinfo_msg->D[0] == 0.0) {
    ROS_ERROR_THROTTLE(5,
                       "Topic '%s' requested but "
                       "camera publishing '%s' is uncalibrated",
                       pub_proc_.getTopic().c_str(),
                       sub_camera_.getInfoTopic().c_str());
    return;
  }

  if (pub_proc_.getNumSubscribers()) {
    const Planck planck = GetPlanck(*cinfo_msg);
    // Get image using cv_bridge
    cv_bridge::CvImagePtr raw_ptr =
        cv_bridge::toCvCopy(image_msg, image_msg->encoding);
    cv::Mat color;
    if (image_msg->encoding == sensor_msgs::image_encodings::MONO8) {
      // Just do a color map conversion for 8 bit
      cv::applyColorMap(raw_ptr->image, color, cv::COLORMAP_JET);
    } else if (image_msg->encoding == sensor_msgs::image_encodings::MONO16) {
      RawToJet(raw_ptr->image, planck, &color);
    } else {
      ROS_ERROR_THROTTLE(5, "Encoding not supported: %s",
                         image_msg->encoding.c_str());
      return;
    }
    cv_bridge::CvImage proc_cvimg(image_msg->header,
                                  sensor_msgs::image_encodings::BGR8, color);
    pub_proc_.publish(proc_cvimg.toImageMsg());
  }
}

void ThermalProcNode::RawToJet(const cv::Mat &raw, const Planck &planck,
                               cv::Mat *color) const {
  const int raw_min = planck.CelsiusToRaw(config_.celsius_min);
  const int raw_max = planck.CelsiusToRaw(config_.celsius_max);
  ROS_ASSERT_MSG(raw_max > raw_min, "max is less than min");
  const double alpha = 255.0 / (raw_max - raw_min);
  const double beta = -alpha * raw_min;
  raw.convertTo(*color, CV_8UC1, alpha, beta);
  cv::applyColorMap(*color, *color, cv::COLORMAP_JET);
}

Planck GetPlanck(const sensor_msgs::CameraInfo &cinfo_msg) {
  return Planck(cinfo_msg.R[0], cinfo_msg.R[1], cinfo_msg.R[2], cinfo_msg.R[3]);
}

void RawToHeat(const cv::Mat &raw, const Planck &planck, cv::Mat *heat) {
  for (int i = 0; i < raw.rows; ++i) {
    float *pheat = heat->ptr<float>(i);
    const uint16_t *praw = raw.ptr<uint16_t>(i);
    for (int j = 0; j < raw.cols; ++j) {
      pheat[j] = static_cast<uint16_t>(planck.RawToCelsius(praw[j]));
    }
  }
}

}  // namespace flir_gige
