#include "flir_gige/flir_gige_ros.h"

namespace flir_gige {

bool FlirGigeRos::Grab(const sensor_msgs::ImagePtr &image_msg) {
  return flir_gige_.GrabImage(*image_msg);
}

}  // namespace flir_gige
