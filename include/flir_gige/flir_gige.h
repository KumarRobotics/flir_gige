#ifndef FLIR_GIGE_GIGE_CAMERA_H_
#define FLIR_GIGE_GIGE_CAMERA_H_

#include <stdint.h>

#include <string>
#include <memory>
#include <thread>
#include <functional>
#include <utility>

#include <PvSystem.h>
#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvDeviceU3V.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvStreamU3V.h>
#include <PvBuffer.h>
#include <PvPipeline.h>

#include <opencv2/core/core.hpp>

#include "flir_gige/planck.h"

namespace flir_gige {

/**
 * @brief The BitSize enum
 */
enum BitSize { BIT8BIT = 2, BIT14BIT };

/**
 * @brief The GigeConfig struct
 */
struct GigeConfig {
  bool color{false};
  int bit{2};  ///< 2 - 8bit, 3 - 14bit
};

/**
 * @brief The FreeDevice struct
 */
struct FreeDevice {
  void operator()(PvDevice *device) const { PvDevice::Free(device); }
};

/**
 * @brief The FreeStream struct
 */
struct FreeStream {
  void operator()(PvStream *stream) const { PvStream::Free(stream); }
};

/**
 * @brief The GigeCamera class
 */
class GigeCamera {
 public:
  GigeCamera(const std::string &ip_address);
  GigeCamera(const GigeCamera &) = delete;             // No copy constructor
  GigeCamera &operator=(const GigeCamera &) = delete;  // No assignment operator

  /**
   * @brief Connect Find and connect to device, create stream and pipeline
   */
  void Connect();

  /**
   * @brief Configure Configure camera before image acquisition
   * @param config
   */
  void Configure(const GigeConfig &config);

  /**
   * @brief Start Start pipeline, enable stream and start acquisition
   */
  void Start();

  /**
   * @brief Stop Stop acquisition, disable stream and stop pipeline
   */
  void Stop();

  /**
   * @brief Disconnect Release all resources we hold
   */
  void Disconnect();

  /**
   * @brief IsAcquire
   * @return true if camera is acquring image
   */
  const bool IsAcquire() const { return acquire_; }

  std::function<void(const cv::Mat &image, const Planck &planck)> use_image;
  std::function<void(const std::pair<double, double> &spot)> use_temperature;

 private:
  using PvDevicePtr = std::unique_ptr<PvDevice, FreeDevice>;
  using PvStreamPtr = std::unique_ptr<PvStream, FreeStream>;
  using PvPipelinePtr = std::unique_ptr<PvPipeline>;
  using ThreadPtr = std::unique_ptr<std::thread>;

  void FindDevice(const std::string &ip);
  void ConnectDevice();
  void OpenStream();
  void ConfigureStream();
  void CreatePipeline();
  void StartAcquisition();
  void StopAcquisition();
  void AcquireImages();
  void LabeledOutput(const std::string &msg) const;

  double GetSpotPixel(const cv::Mat &image) const;
  void SetAoi(const int width, const int height);
  void SetPixelFormat(BitSize bit);

  bool raw_{false};
  bool acquire_{false};
  bool color_{false};  // false - grayscale, true - jet
  std::string label_{"\033[0;35m[ FLIR]:\033[0m "};

  PvSystem system_;
  const PvDeviceInfo *dinfo_;
  PvDevicePtr device_;
  PvStreamPtr stream_;
  PvPipelinePtr pipeline_;
  ThreadPtr image_thread_;
  cv::Mat image_raw_;

};  // class GigeCamera

}  // namespace flir_gige

#endif  // FLIR_GIGE_GIGE_CAMERA_H_
