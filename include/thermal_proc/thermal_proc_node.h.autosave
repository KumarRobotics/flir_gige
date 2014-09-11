#ifndef FLIR_GIGE_THERMAL_PROC_NODE_H_
#define FLIR_GIGE_THERMAL_PROC_NODE_H_

#include <cstdint>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <dynamic_reconfigure/server.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "flir_gige/planck.h"
#include "flir_gige/ThermalProcDynConfig.h"

namespace flir_gige {

class ThermalProcNode {
 public:
  ThermalProcNode(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

 private:
  void CameraCb(const sensor_msgs::ImageConstPtr &image_msg,
                const sensor_msgs::CameraInfoConstPtr &cinfo_msg);
  void ConfigCb(ThermalProcDynConfig &config, int level);
  void ConnectCb();

  void RawToJet(const cv::Mat &raw, const Planck &planck, cv::Mat *color) const;

  ros::NodeHandle nh_, pnh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_camera_;
  image_transport::Publisher pub_proc_;
  dynamic_reconfigure::Server<ThermalProcDynConfig> cfg_server_;
  std::mutex connect_mutex_;
  ThermalProcDynConfig config_;
};

Planck GetPlanck(const sensor_msgs::CameraInfo &cinfo_msg);

void RawToHeat(const cv::Mat &raw, const Planck &planck, cv::Mat *heat);
}  // namespace flir_gige

#endif  // FLIR_GIGE_THERMAL_PROC_NODE_H_
