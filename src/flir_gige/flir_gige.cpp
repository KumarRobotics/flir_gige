#include "flir_gige/flir_gige.h"

#include <cstdint>

#include <algorithm>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <PvGenParameterArray.h>
#include <PvGenParameter.h>

namespace flir_gige {

FlirGige::FlirGige(const std::string &ip_address)
    : ip_address_{ip_address}, dinfo_{nullptr}, ready_{false}, raw_{false} {
  // Find all devices on the network
  const PvResult result = system_.Find();
  if (!result.IsOK()) {
    throw std::runtime_error(std::string("PvSystem::Find Error: ") +
                             result.GetCodeString().GetAscii());
  }
  if (!FindDevice(ip_address)) {
    throw std::runtime_error(ip_address +
                             " not found. Available IP Address(es): " +
                             AvailableDevice());
  }
}

void FlirGige::Connect() {
  ConnectDevice();
  OpenStream();
  ConfigureStream();
  CreatePipeline();
}

void FlirGige::Disconnect() {
  pipeline_.reset();
  stream_.reset();
  device_.reset();
  param_array_ = nullptr;
}

void FlirGige::Configure(FlirGigeDynConfig &config) {
  //  SetPixelFormat(config.raw);
}

bool FlirGige::FindDevice(const std::string &ip) {
  const int interface_cnt = system_.GetInterfaceCount();
  ROS_INFO_STREAM("Interface count:" << interface_cnt);

  // Go through all interfaces, but we only care about network interface
  // For other interfaces such as usb, refer to sample code DeviceFinder.cpp
  std::vector<const PvDeviceInfoGEV *> dinfo_gev_vec;
  for (int i = 0; i < interface_cnt; ++i) {
    // Get pointer to the interface
    const PvInterface *interface = system_.GetInterface(i);
    // Is it a PvNetworkAdapter?
    const auto *nic = dynamic_cast<const PvNetworkAdapter *>(interface);
    if (nic) {
      ROS_INFO("Interface: %s", interface->GetDisplayID().GetAscii());
      // Go through all the devices attached to the network interface
      const int dev_cnt = interface->GetDeviceCount();
      for (int j = 0; j < dev_cnt; ++j) {
        const PvDeviceInfo *dinfo = interface->GetDeviceInfo(j);
        // Is it a GigE Vision device?
        const auto *dinfo_gev = dynamic_cast<const PvDeviceInfoGEV *>(dinfo);
        if (dinfo_gev) {
          ROS_INFO("%d - %s", int(j), dinfo->GetDisplayID().GetAscii());
          dinfo_gev_vec.push_back(dinfo_gev);
        }
      }
    }
  }

  // Check GigE devices found on network adaptor
  if (dinfo_gev_vec.empty()) return false;

  // Try finding the device with the correct ip address
  const auto it = std::find_if(dinfo_gev_vec.cbegin(), dinfo_gev_vec.cend(),
                               [&ip](const PvDeviceInfoGEV *dinfo) {
    return ip == dinfo->GetIPAddress().GetAscii();
  });

  if (it == dinfo_gev_vec.end()) return false;
  // Found device with given ip address
  const PvDeviceInfoGEV *dinfo_gev = *it;
  display_id_ = std::string(dinfo_gev->GetDisplayID().GetAscii());
  ROS_INFO("Found device: %s", display_id().c_str());

  if (!dinfo_gev->IsConfigurationValid()) return false;
  // Try connect and disconnect to verify
  dinfo_ = dinfo_gev;
  PvResult result;
  ROS_INFO("--?-- %s", display_id().c_str());

  // Creates and connects the device controller
  PvDevice *device = PvDevice::CreateAndConnect(dinfo_, &result);
  if (!result.IsOK()) return false;
  ROS_INFO("-->-- %s", display_id().c_str());
  ROS_INFO("--x-- %s", display_id().c_str());
  PvDevice::Free(device);
  return true;
}

std::string FlirGige::AvailableDevice() const {}

void FlirGige::ConnectDevice() {
  ROS_INFO("Connecting to %s", display_id().c_str());
  PvResult result;
  // Use a unique_ptr to manage device resource
  device_.reset(PvDevice::CreateAndConnect(dinfo_, &result));
  if (!result.IsOK()) {
    throw std::runtime_error("Unable to connect to " + display_id());
  }
  param_array_ = device_->GetParameters();
}

void FlirGige::OpenStream() {
  ROS_INFO("Openning stream to %s", display_id().c_str());
  PvResult result;
  // Use a unique_ptr to manage stream resource
  stream_.reset(PvStream::CreateAndOpen(dinfo_->GetConnectionID(), &result));
  if (!stream_) {
    throw std::runtime_error("Unable to stream from " + display_id());
  }
}

void FlirGige::ConfigureStream() {
  // If this is a GigE Vision devie, configure GigE Vision specific parameters
  auto *device_gev = dynamic_cast<PvDeviceGEV *>(device_.get());
  if (!device_gev) {
    throw std::runtime_error("Not a GigE vision device " + display_id());
  }
  ROS_INFO("Configuring gev stream");
  auto *stream_gev = static_cast<PvStreamGEV *>(stream_.get());
  // Negotiate packet size
  device_gev->NegotiatePacketSize();
  // Configure device streaming destination
  device_gev->SetStreamDestination(stream_gev->GetLocalIPAddress(),
                                   stream_gev->GetLocalPort());
}

void FlirGige::CreatePipeline() {
  ROS_INFO("Creating pipeline");
  pipeline_.reset(new PvPipeline(stream_.get()));
  const auto payload_size = device_->GetPayloadSize();
  // Set the Buffer count and the Buffer size
  // BufferCount should be at least 4
  pipeline_->SetBufferCount(4);
  pipeline_->SetBufferSize(payload_size);
}

void FlirGige::StartAcquisition() {
  // Note: the pipeline must be initialized before we start acquisition
  ROS_INFO("Starting acquisition");
  pipeline_->Start();
  device_->StreamEnable();
  param_array_->ExecuteCommand("AcquisitionStart");
}

void FlirGige::StopAcquisition() {
  // Get device parameters need to control streaming
  ROS_INFO("Stop acquisition");
  param_array_->ExecuteCommand("AcquisitionStop");
  device_->StreamDisable();
  pipeline_->Stop();
}

bool FlirGige::GrabImage(sensor_msgs::Image &image_msg) {
  static bool skip_next_frame = false;

  // Start loop for acquisition
  PvBuffer *buffer;
  PvResult op_result;

  // Skip next frame when operation is not ok
  if (skip_next_frame) {
    skip_next_frame = false;
    sleep(1);
  }

  // Retrieve next buffer
  PvResult result = pipeline_->RetrieveNextBuffer(&buffer, 1000, &op_result);

  // Failed to retrieve buffer
  if (result.IsFailure()) {
    ROS_INFO("Retrieve buffer failure");
    return false;
  }

  // Operation not ok, need to return buffer back to pipeline
  if (op_result.IsFailure()) {
    skip_next_frame = true;
    ROS_INFO("Non Ok operation result");
    // Release the buffer back to the pipeline
    pipeline_->ReleaseBuffer(buffer);
    return false;
  }

  // Buffer is not an image
  if ((buffer->GetPayloadType()) != PvPayloadTypeImage) {
    ROS_INFO("Buffer does not contain image");
    pipeline_->ReleaseBuffer(buffer);
    return false;
  }

  // Get image specific buffer interface
  PvImage *image = buffer->GetImage();

  // Get device parameters need to control streaming
  int64_t width{0}, height{0};
  param_array_->GetIntegerValue("Width", width);
  param_array_->GetIntegerValue("Height", height);

  // Assemble image msg
  image_msg.height = height;
  image_msg.width = width;
  image_msg.step = image_msg.width;
  if (raw_) {
    image_msg.encoding = sensor_msgs::image_encodings::MONO16;
  } else {
    image_msg.encoding = sensor_msgs::image_encodings::MONO8;
  }

  const size_t data_size = image->GetImageSize();
  if (image_msg.data.size() != data_size) {
    image_msg.data.resize(data_size);
  }
  memcpy(&image_msg.data[0], image->GetDataPointer(), image->GetImageSize());

  // Release the buffer back to the pipeline
  pipeline_->ReleaseBuffer(buffer);
  return true;
}

// This function is not intended to be used
void FlirGige::SetAoi(int *width, int *height) {
  // Get current width and height
  int64_t curr_width = 0;
  int64_t curr_height = 0;
  param_array_->GetIntegerValue("Width", curr_width);
  param_array_->GetIntegerValue("Height", curr_height);
  // Check to see if it's necessary to change width and height
  if (curr_width != *width) {
    param_array_->SetIntegerValue("Width", *width);
  }
  if (curr_height != *height) {
    param_array_->SetIntegerValue("Height", *height);
  }
}

// void FlirGige::SetPixelFormat(int bit) {
//  PvGenParameterArray *device_params = device_->GetParameters();
//  int64_t height = 0, width = 0;
//  device_params->GetIntegerValue("Width", width);
//  device_params->GetIntegerValue("Height", height);
//  // Set digital output and pixel format
//  if (bit == BIT8BIT) {
//    device_params->SetEnumValue("PixelFormat", PvPixelMono8);
//    device_params->SetEnumValue("DigitalOutput", static_cast<int64_t>(bit));
//  } else if (bit == BIT14BIT) {
//    device_params->SetEnumValue("PixelFormat", PvPixelMono14);
//    device_params->SetEnumValue("DigitalOutput", static_cast<int64_t>(bit));
//  }
//  // Verify setting
//  PvString digital_output;
//  device_params->GetEnumValue("DigitalOutput", digital_output);
//}

// double FlirGige::GetSpotPixel(const cv::Mat &image) const {
//  auto c = image.cols / 2;
//  auto r = image.rows / 2;
//  auto s1 = image.at<uint16_t>(r - 1, c - 1);
//  auto s2 = image.at<uint16_t>(r - 1, c);
//  auto s3 = image.at<uint16_t>(r, c - 1);
//  auto s4 = image.at<uint16_t>(r, c);
//  return static_cast<double>(s1 / 4 + s2 / 4 + s3 / 4 + s4 / 4);
//}

}  // namespace flir_gige
