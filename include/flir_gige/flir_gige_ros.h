#ifndef FLIR_GIGE_ROS_H_
#define FLIR_GIGE_ROS_H_

#include "flir_gige/flir_gige.h"
#include "camera_base/camera_ros_base.h"

namespace flir_gige {

class FlirGigeRos : public CameraRosBase {
 public:
  FlirGigeRos(const ros::NodeHandle& nh)
      : CameraRosBase{nh}, flir_gige_{identifier()} {}

  virtual bool Grab(const sensor_msgs::ImagePtr* image_msg) override;

 private:
  FlirGige flir_gige_;
};

}  // namespace flir_gige

#endif  // FLIR_GIGE_ROS_H_
