#include "flir_gige/flir_gige_node.h"

namespace flir_gige {

void FlirGigeNode::Setup(FlirGigeDynConfig &config) {
  flir_gige_ros_.set_fps(config.fps);
  flir_gige_ros_.Reconnect();
  flir_gige_ros_.camera().Configure(config);
  flir_gige_ros_.Start();
}

void FlirGigeNode::Acquire() {
  while (is_acquire() && ros::ok()) {
    flir_gige_ros_.Publish(ros::Time::now());
    Sleep();
  }
}

}  // namespace flir_gige
