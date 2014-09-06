#include "thermal_proc/thermal_proc_node.h"

#include <memory>
#include <thread>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace flir_gige {

class ThermalProcNodelet : public nodelet::Nodelet {
 public:
  ThermalProcNodelet() : nodelet::Nodelet() {}
  ~ThermalProcNodelet() {}

  virtual void onInit() {
    try {
      thermal_proc_node_.reset(
          new ThermalProcNode(getPrivateNodeHandle(), getPrivateNodeHandle()));
    }
    catch (const std::exception &e) {
      NODELET_ERROR("%s: %s", getPrivateNodeHandle().getNamespace().c_str(),
                    e.what());
    }
  }

 private:
  std::unique_ptr<ThermalProcNode> thermal_proc_node_;
};

PLUGINLIB_EXPORT_CLASS(ThermalProcNodelet, nodelet::Nodelet)

}  // namespace flir_gige
