#include "flir_gige/gige_camera.h"

#include <cmath>
#include <unistd.h>

#include <iostream>
#include <stdexcept>
#include <sstream>
#include <vector>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <PvGenParameterArray.h>
#include <PvGenParameter.h>

namespace flir_gige {

GigeCamera::GigeCamera(const std::string &ip_address) {
  // Find all devices on the network
  PvResult result = system_.Find();
  if (!result.IsOK()) {
    std::ostringstream error_msg;
    error_msg << "PvSystem::Find Error: " << result.GetCodeString().GetAscii();
    throw std::runtime_error(error_msg.str());
  }
  FindDevice(ip_address);
}

void GigeCamera::Connect() {
  ConnectDevice();
  OpenStream();
  ConfigureStream();
  CreatePipeline();
}

void GigeCamera::Disconnect() {
  // Release all the recourses we hold
  pipeline_.reset();
  stream_.reset();
  device_.reset();
}

void GigeCamera::Configure(const GigeConfig &config) {
  color_ = config.color;
  SetPixelFormat(static_cast<BitSize>(config.bit));
  //  SetAoi(config.width, config.height);
}

void GigeCamera::Start() {
  StartAcquisition();
  // Set acquire to true
  acquire_ = true;
  // Creat a new thread for acquisition
  image_thread_.reset(new std::thread(&GigeCamera::AcquireImages, this));
}

void GigeCamera::Stop() {
  // Set acquire to false to stop the thread
  acquire_ = false;
  // Wait for the thread to finish
  image_thread_->join();
  StopAcquisition();
}

void GigeCamera::FindDevice(const std::string &ip) {
  auto interface_count = system_.GetInterfaceCount();
  std::cout << label_ << "Interface count: " << interface_count << std::endl;

  // Go through all interfaces, but we only care about network interface
  // For other interfaces such as usb, refer to sample code DeviceFinder.cpp
  std::vector<const PvDeviceInfoGEV *> dinfo_gev_vec;
  for (decltype(interface_count) i = 0; i < interface_count; ++i) {
    // Get pointer to the interface
    const PvInterface *interface = system_.GetInterface(i);
    // Is it a PvNetworkAdapter?
    const auto *nic = dynamic_cast<const PvNetworkAdapter *>(interface);
    if (nic) {
      std::cout << label_ << i << " - " << interface->GetDisplayID().GetAscii()
                << std::endl;
      // Go through all the devices attached to the network interface
      for (uint32_t j = 0; j < interface->GetDeviceCount(); ++j) {
        const PvDeviceInfo *dinfo = interface->GetDeviceInfo(j);
        // Is it a GigE Vision device?
        const auto *dinfo_gev = dynamic_cast<const PvDeviceInfoGEV *>(dinfo);
        if (dinfo_gev) {
          std::cout << label_ << "  " << j << " - "
                    << dinfo->GetDisplayID().GetAscii() << std::endl;
          dinfo_gev_vec.push_back(dinfo_gev);
        }
      }
    }
  }

  // Check GigE devices found on network adaptor
  if (dinfo_gev_vec.empty()) {
    throw std::runtime_error("GigeCamera: No device found");
  }
  // Try finding the device with the correct ip address
  const auto it = std::find_if(dinfo_gev_vec.cbegin(), dinfo_gev_vec.cend(),
                               [ip](const PvDeviceInfoGEV *dinfo) {
    return ip == dinfo->GetIPAddress().GetAscii();
  });

  if (it == dinfo_gev_vec.end()) {
    // Did not find device with given ip address
    std::ostringstream error_msg;
    error_msg << "GigeCamera: Device not found. Available IP Address:";
    for (const PvDeviceInfoGEV *dinfo : dinfo_gev_vec) {
      error_msg << " " << dinfo->GetIPAddress().GetAscii();
    }
    throw std::runtime_error(error_msg.str());
  } else {
    // Found device with given ip address
    LabeledOutput(std::string("Found device: ") +
                  (*it)->GetIPAddress().GetAscii());
    // Is the IP address valid?
    if ((*it)->IsConfigurationValid()) {
      // Try connect and disconnect to verify
      dinfo_ = *it;
      PvResult result;
      LabeledOutput(std::string("--?-- ") + dinfo_->GetDisplayID().GetAscii());
      // Creates and connects the device controller
      PvDevice *device = PvDevice::CreateAndConnect(dinfo_, &result);
      if (result.IsOK()) {
        std::cout << label_ << "-->-- " << dinfo_->GetDisplayID().GetAscii()
                  << std::endl;
        std::cout << label_ << "--x-- " << dinfo_->GetDisplayID().GetAscii()
                  << std::endl;
        PvDevice::Free(device);
      } else {
        // Maybe throw an exception here?
        std::ostringstream error_msg;
        error_msg << "GigeCamera: Unable to connect to "
                  << dinfo_->GetDisplayID().GetAscii();
        throw std::runtime_error(error_msg.str());
      }
    }
  }
}

void GigeCamera::ConnectDevice() {
  std::cout << label_ << "Connecting to " << dinfo_->GetDisplayID().GetAscii();
  PvResult result;
  // Use a unique_ptr to manage device resource
  device_.reset(PvDevice::CreateAndConnect(dinfo_, &result));

  if (result.IsOK()) {
    std::cout << " ... Done." << std::endl;
  } else {
    std::ostringstream error_msg;
    error_msg << "GigeCamera: Unable to connect to "
              << dinfo_->GetDisplayID().GetAscii();
    throw std::runtime_error(error_msg.str());
  }
}

void GigeCamera::OpenStream() {
  std::cout << label_ << "Opening stream to "
            << dinfo_->GetDisplayID().GetAscii();
  PvResult result;
  // Use a unique_ptr to manage stream resource
  stream_.reset(PvStream::CreateAndOpen(dinfo_->GetConnectionID(), &result));

  if (stream_) {
    std::cout << " ... Done. " << std::endl;
  } else {
    // Maybe a function for throw exception?
    std::ostringstream error_msg;
    error_msg << "GigeCamera: Unable to stream form "
              << dinfo_->GetDisplayID().GetAscii();
    throw std::runtime_error(error_msg.str());
  }
}

void GigeCamera::ConfigureStream() {
  // If this is a GigE Vision devie, configure GigE Vision specific parameters
  auto *device_gev = dynamic_cast<PvDeviceGEV *>(device_.get());
  if (device_gev) {
    LabeledOutput("Configuring gev stream");
    auto *stream_gev = static_cast<PvStreamGEV *>(stream_.get());
    // Negotiate packet size
    device_gev->NegotiatePacketSize();
    // Configure device streaming destination
    device_gev->SetStreamDestination(stream_gev->GetLocalIPAddress(),
                                     stream_gev->GetLocalPort());
  } else {
    std::ostringstream error_msg;
    error_msg << "GigeCamera: This is not a GigE Vision device "
              << dinfo_->GetDisplayID().GetAscii();
    throw std::runtime_error(error_msg.str());
  }
}

void GigeCamera::CreatePipeline() {
  LabeledOutput("Creating pipeline");
  // Create the PvPipeline object
  pipeline_.reset(new PvPipeline(stream_.get()));
  // Reading payload size from device
  auto payload_size = device_->GetPayloadSize();
  // Set the Buffer count and the Buffer size
  // BufferCount should be at least 4
  pipeline_->SetBufferCount(4);
  pipeline_->SetBufferSize(payload_size);
}

void GigeCamera::StartAcquisition() {
  PvGenParameterArray *device_params = device_->GetParameters();
  // Note: the pipeline must be initialized before we start acquisition
  std::cout << label_ << "Starting pipeline ... ";
  pipeline_->Start();
  // Enable streaming
  std::cout << "Enabling streaming ... ";
  device_->StreamEnable();
  // Start acquisition
  std::cout << "Start acquisition ... ";
  // Get device parameters need to control streaming
  device_params->ExecuteCommand("AcquisitionStart");
  std::cout << "Done" << std::endl;
}

void GigeCamera::StopAcquisition() {
  // Get device parameters need to control streaming
  PvGenParameterArray *device_params = device_->GetParameters();
  // Stop image acquisition
  std::cout << label_ << "Stop acquisition ... ";
  device_params->ExecuteCommand("AcquisitionStop");
  // Get controller out of streaming
  std::cout << "Disabling streaming ... ";
  device_->StreamDisable();
  // Stop pipeline
  std::cout << "Stoping pipeline ... ";
  pipeline_->Stop();
  std::cout << "Done" << std::endl;
}

void GigeCamera::AcquireImages() {
  // Get device parameters need to control streaming
  PvGenParameterArray *device_params = device_->GetParameters();
  int64_t width{0}, height{0};
  int64_t R{0};
  double F{0.0}, B{0.0}, O{0.0};
  double spot{0};
  device_params->GetIntegerValue("Width", width);
  device_params->GetIntegerValue("Height", height);
  device_params->GetIntegerValue("R", R);
  device_params->GetFloatValue("F", F);
  device_params->GetFloatValue("B", B);
  device_params->GetFloatValue("O", O);
  std::cout << label_ << "R: " << R << " F: " << F << " B: " << B << " O: " << O
            << std::endl;
  const Planck planck(B, F, O, R);

  bool skip_next_frame = false;

  // Start loop for acquisition
  while (acquire_) {
    PvBuffer *buffer;
    PvResult op_result;

    // Skip next frame when operation is not ok
    if (skip_next_frame) {
      skip_next_frame = false;
      sleep(1);
      continue;
    }

    // Retrieve next buffer
    PvResult result = pipeline_->RetrieveNextBuffer(&buffer, 1000, &op_result);

    // Failed to retrieve buffer
    if (result.IsFailure()) {
      LabeledOutput("Retrieve buffer failure");
      continue;
    }

    // Operation not ok, need to return buffer back to pipeline
    if (op_result.IsFailure()) {
      skip_next_frame = true;
      LabeledOutput("Non Ok operation result");
      // Release the buffer back to the pipeline
      pipeline_->ReleaseBuffer(buffer);
      continue;
    }

    // Buffer is not an image
    if ((buffer->GetPayloadType()) != PvPayloadTypeImage) {
      LabeledOutput("Buffer does not contain image");
      pipeline_->ReleaseBuffer(buffer);
      continue;
    }

    // Get image specific buffer interface
    PvImage *image = buffer->GetImage();
    memcpy(image_raw_.data, image->GetDataPointer(), image->GetImageSize());
    // Use the image for temperature calculation only in raw data mode
    if (raw_) {
      device_params->GetFloatValue("Spot", spot);
      double t = planck.RawToCelsius(GetSpotPixel(image_raw_));
      use_temperature(std::make_pair(spot, t));
      use_image(image_raw_, planck);
    } else {
      // For display purpose in non raw data mode
      if (color_) {
        cv::Mat image_color;
        cv::applyColorMap(image_raw_, image_color, cv::COLORMAP_JET);
        use_image(image_color, planck);
      } else {
        use_image(image_raw_, planck);
      }
    }
    // Release the buffer back to the pipeline
    pipeline_->ReleaseBuffer(buffer);
  }
}

void GigeCamera::SetAoi(const int width, const int height) {
  PvGenParameterArray *device_params = device_->GetParameters();
  // Get width and height parameter
  auto *width_param = dynamic_cast<PvGenInteger *>(device_params->Get("Width"));
  auto *height_param =
      dynamic_cast<PvGenInteger *>(device_params->Get("Height"));
  // Get current width and height
  int64_t current_width = 0;
  int64_t current_height = 0;
  width_param->GetValue(current_width);
  height_param->GetValue(current_height);
  // Check to see if it's necessary to change width and height
  if (current_width != width) {
    if (width_param->SetValue(width).IsFailure()) {
      LabeledOutput("failed to set width");
    }
  }
  if (current_height != height) {
    if (height_param->SetValue(height).IsFailure()) {
      LabeledOutput("failed to set height");
    }
  }
}

void GigeCamera::SetPixelFormat(BitSize bit) {
  PvGenParameterArray *device_params = device_->GetParameters();
  int64_t height = 0, width = 0;
  device_params->GetIntegerValue("Width", width);
  device_params->GetIntegerValue("Height", height);
  // Set digital output and pixel format
  if (bit == BIT8BIT) {
    device_params->SetEnumValue("PixelFormat", PvPixelMono8);
    device_params->SetEnumValue("DigitalOutput", static_cast<int64_t>(bit));
    image_raw_.create(cv::Size(width, height), CV_8UC1);
    raw_ = false;
  } else if (bit == BIT14BIT) {
    device_params->SetEnumValue("PixelFormat", PvPixelMono14);
    device_params->SetEnumValue("DigitalOutput", static_cast<int64_t>(bit));
    image_raw_.create(cv::Size(width, height), CV_16UC1);
    raw_ = true;
  }
  // Verify setting
  PvString digital_output;
  device_params->GetEnumValue("DigitalOutput", digital_output);
  std::cout << label_ << "Width: " << width << " Height: " << height
            << " Bit: " << digital_output.GetAscii() << std::endl;
}

double GigeCamera::GetSpotPixel(const cv::Mat &image) const {
  auto c = image.cols / 2;
  auto r = image.rows / 2;
  auto s1 = image.at<uint16_t>(r - 1, c - 1);
  auto s2 = image.at<uint16_t>(r - 1, c);
  auto s3 = image.at<uint16_t>(r, c - 1);
  auto s4 = image.at<uint16_t>(r, c);
  return static_cast<double>(s1 / 4 + s2 / 4 + s3 / 4 + s4 / 4);
}

void GigeCamera::LabeledOutput(const std::string &msg) const {
  std::cout << label_ << msg << std::endl;
}

}  // namespace flir_gige
