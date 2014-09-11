#ifndef FLIR_GIGE_CALIB_PROC_NODE_H_
#define FLIR_GIGE_CALIB_PROC_NODE_H_

#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <flir_gige/CalibProcDynConfig.h>

#include <opencv2/core/core.hpp>

namespace flir_gige {

class CalibProcNode {
 public:
  CalibProcNode(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

 private:
  void ImageCb(const sensor_msgs::ImageConstPtr &image_msg);
  void ConnectCb();
  void ConfigCb(CalibProcDynConfig &config, int level);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_image_;
  image_transport::Publisher pub_calib_;
  std::mutex connect_mutex_;
  dynamic_reconfigure::Server<CalibProcDynConfig> cfg_server_;
  CalibProcDynConfig config_;
};

void DetectAndDrawCriclesGrid(const cv::Mat &src, const cv::Size &size,
                              cv::Mat &disp);

}  // namespace flir_gige

#endif  // FLIR_GIGE_CALIB_PROC_NODE_H_
