#ifndef FLIR_GIGE_H_
#define FLIR_GIGE_H_

#include <memory>

#include <PvSystem.h>
#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvDeviceU3V.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvStreamU3V.h>
#include <PvBuffer.h>
#include <PvPipeline.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Temperature.h>
#include <flir_gige/FlirGigeDynConfig.h>

#include "flir_gige/planck.h"

namespace flir_gige {

struct FreeDevice {
  void operator()(PvDevice *device) const { PvDevice::Free(device); }
};

struct FreeStream {
  void operator()(PvStream *stream) const { PvStream::Free(stream); }
};

class FlirGige {
 public:
  FlirGige(const std::string &ip_address);

  const std::string &ip_address() const { return ip_address_; }
  const std::string &display_id() const { return display_id_; }

  void Connect();
  void Disconnect();
  void StartAcquisition();
  void StopAcquisition();
  void Configure(FlirGigeDynConfig &config);
  bool GrabImage(sensor_msgs::Image &image_msg,
                 sensor_msgs::CameraInfo &cinfo_msg);
  bool GrabTemprature(sensor_msgs::Temperature &temp_msg);

 private:
  using PvDevicePtr = std::unique_ptr<PvDevice, FreeDevice>;
  using PvStreamPtr = std::unique_ptr<PvStream, FreeStream>;
  using PvPipelinePtr = std::unique_ptr<PvPipeline>;
  using PvDeviceInfoGEVVec = std::vector<const PvDeviceInfoGEV *>;

  bool FindDevice(const std::string &ip,
                  const PvDeviceInfoGEVVec &dinfo_gev_vec);
  std::string AvailableDevice(const PvDeviceInfoGEVVec &dinfo_gev_vec) const;
  PvDeviceInfoGEVVec GatherGevDevice() const;

  void ConnectDevice();
  void OpenStream();
  void ConfigureStream();
  void CreatePipeline();
  void CacheParams();

  void SetAoi(int *width, int *height) const;
  void SetPixelFormat(bool raw) const;
  void SetNucMode(int nuc) const;
  void DoNuc(bool& nuc) const;

  //  double GetSpotPixel(const cv::Mat &image) const;

  std::string ip_address_;
  std::string display_id_;
  PvSystem system_;
  const PvDeviceInfo *dinfo_;
  PvDevicePtr device_;
  PvStreamPtr stream_;
  PvPipelinePtr pipeline_;
  PvGenParameterArray *param_array_;
  struct {
    int height;
    int width;
    double B, F, O, R;
    int bit;
  } cache_;
};

}  // namespace flir_gige

#endif  // FLIR_GIGE_H_
