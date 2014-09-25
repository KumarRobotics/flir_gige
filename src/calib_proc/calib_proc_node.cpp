#include "calib_proc/calib_proc_node.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace flir_gige {

CalibProcNode::CalibProcNode(const ros::NodeHandle &nh,
                             const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), it_(nh), cfg_server_(pnh) {
  image_transport::SubscriberStatusCallback connect_cb =
      boost::bind(&CalibProcNode::ConnectCb, this);
  pub_calib_ = it_.advertise("image_calib", 1, connect_cb, connect_cb);
  cfg_server_.setCallback(boost::bind(&CalibProcNode::ConfigCb, this, _1, _2));
}

void CalibProcNode::ConnectCb() {
  std::lock_guard<std::mutex> lock(connect_mutex_);
  if (!pub_calib_.getNumSubscribers()) {
    sub_image_.shutdown();
  } else if (!sub_image_) {
    image_transport::TransportHints hints("raw", ros::TransportHints(), nh_);
    sub_image_ =
        it_.subscribe("image_raw", 2, &CalibProcNode::ImageCb, this, hints);
  }
}

void CalibProcNode::ConfigCb(CalibProcDynConfig &config, int level) {
  if (level < 0) {
    ROS_INFO("%s: initialize dynamic reconfigure server",
             pnh_.getNamespace().c_str());
  }
  if (!(config.thresh_window % 2)) {
    config.thresh_window += 1;
  }
  if (config.min_area > config.max_area) {
    config.max_area = config.min_area + 1;
  }
  config_ = config;
}

void CalibProcNode::ImageCb(const sensor_msgs::ImageConstPtr &image_msg) {
  cv::Mat image = cv_bridge::toCvCopy(image_msg, image_msg->encoding)->image;

  cv::Mat inverted;
  cv::bitwise_not(image, inverted);
  /*
    // Threshold
    cv::Mat thresh;
    cv::adaptiveThreshold(image, thresh, 255, config_.thresh_type,
                          cv::THRESH_BINARY, config_.thresh_window, 0);
    cv::imshow("thresh", thresh);

    // Gaussian blur
    cv::Mat gauss;
    cv::GaussianBlur(thresh, gauss, cv::Size(), config_.sigma, config_.sigma,
                     cv::BORDER_DEFAULT);
    cv::imshow("gauss", gauss);

    // Erosion
    cv::Mat eroded;
    int erosion_size = config_.erosion_size;
    cv::Mat element = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
        cv::Point(erosion_size, erosion_size));
    cv::erode(thresh, eroded, element);
    cv::imshow("erosion", eroded);

    // Contour
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(eroded, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    cv::Mat ctr_image;
    cv::cvtColor(image, ctr_image, CV_GRAY2BGR);
    for (size_t i = 0; i < contours.size(); ++i) {
      double area = cv::contourArea(contours[i]);
      if (area > config_.min_area && area < config_.max_area) {
        cv::drawContours(ctr_image, contours, i, cv::Scalar(255, 0, 0), 2, 8);
      }
    }
    cv::imshow("contour", ctr_image);
    */

  //  cv::Mat display;
  //  DetectAndDrawCriclesGrid(
  //      inverted, cv::Size(config_.circles_per_row, config_.circles_per_col),
  //      display);

  // Display
  //  cv::imshow("display", display);
  //  cv::waitKey(1);

  cv::Mat calib(inverted);
  // Publish processed image
  cv_bridge::CvImage cvimg_calib(image_msg->header, image_msg->encoding, calib);
  pub_calib_.publish(cvimg_calib.toImageMsg());
}

void DetectAndDrawCriclesGrid(const cv::Mat &src, const cv::Size &size,
                              cv::Mat &disp) {
  std::vector<cv::Point2f> centers;
  bool found = cv::findCirclesGrid(src, size, centers);
  if (disp.empty()) {
    disp = src.clone();
  }
  if (disp.channels() == 1) {
    cv::cvtColor(disp, disp, CV_GRAY2BGR);
  }
  cv::drawChessboardCorners(disp, size, cv::Mat(centers), found);
}
}  // namespace flir_gige
