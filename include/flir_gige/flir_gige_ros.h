#ifndef FLIR_GIGE_ROS_H_
#define FLIR_GIGE_ROS_H_

#include "flir_gige/flir_gige.h"
#include "camera_base/camera_ros_base.h"

namespace flir_gige {

class FlirGigeRos : public CameraRosBase {
 public:
  FlirGigeRos(const ros::NodeHandle& nh)
      : CameraRosBase{nh}, flir_gige_{identifier()} {
    SetHardwareId(flir_gige_.display_id());
  }

  FlirGige& camera() { return flir_gige_; }

  void Reconnect() {
    flir_gige_.StopAcquisition();
    flir_gige_.Disconnect();
    flir_gige_.Connect();
  }
  void Start() { flir_gige_.StartAcquisition(); }

  virtual bool Grab(const sensor_msgs::ImagePtr& image_msg) override;

 private:
  FlirGige flir_gige_;
};

}  // namespace flir_gige

#endif  // FLIR_GIGE_ROS_H_
