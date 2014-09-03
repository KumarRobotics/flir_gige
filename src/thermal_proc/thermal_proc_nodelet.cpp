#include "flir_gige/thermal_proc/thermal_proc.h"

#include <memory>
#include <thread>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace flir_gige {

class ThermalProcNodelet : public nodelet::Nodelet {
 public:
  ThermalProcNodelet() : nodelet::Nodelet() {}
  ~ThermalProcNodelet() {}

  virtual void onInit() {
    ROS_WARN_STREAM("Starting");
    try {
      thermal_proc_.reset(
          new ThermalProc(getPrivateNodeHandle(), getPrivateNodeHandle()));
    }
    catch (const std::exception &e) {
      ROS_ERROR_STREAM("ThermalProc: " << e.what());
    }
  }

 private:
  std::shared_ptr<ThermalProc> thermal_proc_;
};

PLUGINLIB_DECLARE_CLASS(flir_gige, ThermalProcNodelet,
                        flir_gige::ThermalProcNodelet, nodelet::Nodelet)
}  // namespace flir_gige
