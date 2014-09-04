#include "flir_gige/flir_gige_node.h"

namespace flir_gige {

void FlirGigeNode::Setup(FlirGigeDynConfig &config) {
  flir_gige_ros_.camera().StopAcquisition();
  flir_gige_ros_.camera().Disconnect();
  flir_gige_ros_.camera().Connect();
  flir_gige_ros_.camera().Configure(config);
  flir_gige_ros_.camera().StartAcquisition();
}

void FlirGigeNode::Acquire() {
  while (is_acquire() && ros::ok()) {
    flir_gige_ros_.Publish(ros::Time::now());
    Sleep();
  }
}

// FlirGige::FlirGige(const ros::NodeHandle &nh)
//      pub_temperature_{nh_.advertise<sensor_msgs::Temperature>("spot", 1)}

/*
void FlirGige::Run() {
  GigeConfig config;
  nh_.param<bool>("color", config.color, config.color);
  nh_.param<int>("bit", config.bit, config.bit);
  gige_camera_->Connect();
  gige_camera_->Configure(config);
  gige_camera_->Start();
}
*/

/*
void FlirGige::End() {
  gige_camera_->Stop();
  gige_camera_->Disconnect();
}
*/

/*
void FlirGige::PublishImage(const cv::Mat &image, const Planck &planck) {
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = frame_id_;
  // We put planck constants into CameraInfo::R. The orders are B F O R
  cinfo_->R[0] = planck.B;
  cinfo_->R[1] = planck.F;
  cinfo_->R[2] = planck.O;
  cinfo_->R[3] = planck.R;
  // Convert to ros image msg and publish camera
  cv_bridge::CvImage cv_image(header, GetImageEncoding(image), image);
  image_ = cv_image.toImageMsg();
  cinfo_->header = header;
  pub_camera_.publish(image_, cinfo_);
  rate_->sleep();
}
*/

/*
void FlirGige::PublishTemperature(const std::pair<double, double> &spot) {
  // Construct a temperature mesage
  sensor_msgs::Temperature temperature;
  temperature.header.stamp = ros::Time::now();
  temperature.header.frame_id = frame_id_;
  temperature.temperature = spot.first;
  temperature.variance = spot.second;
  pub_temperature_.publish(temperature);
}
*/

/*
std::string FlirGige::GetImageEncoding(const cv::Mat &image) const {
  std::string encoding;
  switch (image.type()) {
    case CV_8UC1:
      encoding = sensor_msgs::image_encodings::MONO8;
      break;
    case CV_8UC3:
      encoding = sensor_msgs::image_encodings::BGR8;
      break;
    case CV_16UC1:
      encoding = sensor_msgs::image_encodings::MONO16;
      break;
    default:
      encoding = sensor_msgs::image_encodings::MONO8;
  }
  return encoding;
}
*/

/*
void FlirGige::ConfigCb(FlirDynConfig &config, int level) {
  // Do nothing when first starting
  if (level < 0) {
    return;
  }
  // Get config
  GigeConfig gige_config;
  // Color image only works with 8-bit output
  if (config.color) config.bit = 2;
  gige_config.color = config.color;
  gige_config.bit = config.bit;
  // Stop the camera if in acquisition
  if (gige_camera_->IsAcquire()) {
    // Stop the image thread if camera is running
    gige_camera_->Stop();
    gige_camera_->Disconnect();
  }
  gige_camera_->Connect();
  gige_camera_->Configure(gige_config);
  gige_camera_->Start();
}
*/

}  // namespace flir_gige
