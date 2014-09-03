#include "flir_gige/flir_gige_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "flir_gige_node");
  ros::NodeHandle nh("~");

  try {
    flir_gige::FlirGigeNode flir_gige_node(nh);
    flir_gige_node.Run();
    ros::spin();
    flir_gige_node.End();
  }
  catch (const std::exception &e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}
