#include "flir_gige/flir_gige_ros.h"

namespace flir_gige {

bool FlirGigeRos::Grab(const sensor_msgs::ImagePtr &image_msg) {
  // Add expose time to current time stamp
  //  image_msg->header.stamp += ros::Duration(bluefox2_.expose_us() * 1e-6);
  //  return bluefox2_.GrabImage(*image_msg);
  return false;
}

}  // namespace flir_gige
