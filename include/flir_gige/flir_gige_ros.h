#ifndef FLIR_GIGE_FLIR_GIGE_H_
#define FLIR_GIGE_FLIR_GIGE_H_

#include <utility>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <opencv2/core/core.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include "flir_gige/planck.h"
#include "flir_gige/gige_camera.h"
#include "flir_gige/FlirDynConfig.h"

namespace flir_gige {

class FlirGige {
 public:
  using camera_info_manager::CameraInfoManager;
  using CameraInfoManagerPtr = boost::shared_ptr<CameraInfoManager>;
  using FlirDynConfig = ::flir_gige::FlirDynConfig;

  /**
   * @brief FlirGige Constructor
   * @param nh Private node handle
   */
  FlirGige(const ros::NodeHandle &nh);

  /**
   * @brief Run Run camera
   */
  void Run();

  /**
   * @brief End End camera
   */
  void End();

 private:
  void ReadConfig();
  void PublishImage(const cv::Mat &image, const Planck &planck);
  void PublishTemperature(const std::pair<double, double> &spot);
  std::string GetImageEncoding(const cv::Mat &image) const;
  void ConfigCb(FlirDynConfig &config, int level);

  ros::NodeHandle nh_;                 ///< Private node handle
  std::string frame_id_;               ///< Frame id
  std::string camera_name_;            ///< Camera name
  boost::shared_ptr<ros::Rate> rate_;  ///< Acquisition rate

  image_transport::ImageTransport it_;           ///< Image transport
  image_transport::CameraPublisher pub_camera_;  ///< Camera publisher
  CameraInfoManagerPtr cinfo_manager_;           ///< Camera info manager
  ros::Publisher pub_temperature_;               ///< Temperature publisher

  FlirDynConfig config_;  ///< Reconfigure parameters
  dynamic_reconfigure::Server<FlirDynConfig> server_;  ///< Reconfigure server

  diagnostic_updater::Updater updater_;              ///< Diagnostic updater
  diagnostic_updater::TopicDiagnostic diagnostics_;  ///< topic diagnostics

  boost::shared_ptr<GigeCamera> gige_camera_;      ///< GigE camera
  boost::shared_ptr<boost::thread> image_thread_;  ///< Image acquisition thread
};

}  // namespace flir_gige

#endif  // FLIR_GIGE_FLIR_GIGE_H_
