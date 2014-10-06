#ifndef FLIR_GIGE_ROS_H_
#define FLIR_GIGE_ROS_H_

#include "flir_gige/flir_gige.h"
#include "camera_base/camera_ros_base.h"

#include <sensor_msgs/Temperature.h>

namespace flir_gige {

class FlirGigeRos : public camera_base::CameraRosBase {
 public:
  FlirGigeRos(const ros::NodeHandle& nh)
      : CameraRosBase(nh),
        flir_gige_(identifier()),
        nh_(nh),
        temp_pub_(nh_.advertise<sensor_msgs::Temperature>("spot", 1)),
        temp_msg_(new sensor_msgs::Temperature()) {
    SetHardwareId(flir_gige_.display_id());
  }

  FlirGige& camera() { return flir_gige_; }

  void Reconnect() {
    flir_gige_.StopAcquisition();
    flir_gige_.Disconnect();
    flir_gige_.Connect();
  }
  void Start() { flir_gige_.StartAcquisition(); }

  virtual bool Grab(const sensor_msgs::ImagePtr& image_msg,
                    const sensor_msgs::CameraInfoPtr& cinfo_msg) override;

  void PublishTemperature(const ros::Time& time);

 private:
  FlirGige flir_gige_;
  ros::NodeHandle nh_;
  ros::Publisher temp_pub_;
  sensor_msgs::TemperaturePtr temp_msg_;
};

}  // namespace flir_gige

#endif  // FLIR_GIGE_ROS_H_
