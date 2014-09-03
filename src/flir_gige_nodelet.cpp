#include "flir_gige/flir_gige_node.h"

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace flir_gige {

class FlirNodelet : public nodelet::Nodelet {
 public:
  FlirNodelet() : nodelet::Nodelet() {}
  ~FlirNodelet() { flir_gige_node_->End(); }

  virtual void onInit() {
    try {
      flir_gige_node_.reset(new FlirGigeNode(getPrivateNodeHandle()));
      flir_gige_node_->Run();
    }
    catch (const std::exception &e) {
      NODELET_ERROR("%s: %s", getPrivateNodeHandle().getNamespace().c_str(),
                    e.what());
    }
  }

 private:
  std::unique_ptr<FlirGigeNode> flir_gige_node_;
};

PLUGINLIB_DECLARE_CLASS(flir_gige, FlirNodelet, flir_gige::FlirNodelet,
                        nodelet::Nodelet)

}  // namespace flir_gige
