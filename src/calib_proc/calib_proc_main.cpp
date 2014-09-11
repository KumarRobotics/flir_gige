#include "calib_proc/calib_proc_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "calib_proc");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    flir_gige::CalibProcNode calib_proc_node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception &e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
}
