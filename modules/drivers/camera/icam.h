#pragma once

#include <malloc.h>

#include "cyber/cyber.h"

#include "modules/drivers/camera/proto/config.pb.h"

namespace apollo {
namespace drivers {
namespace camera {

using apollo::drivers::camera::config::Config;
using apollo::drivers::camera::config::IO_METHOD_MMAP;
using apollo::drivers::camera::config::IO_METHOD_READ;
using apollo::drivers::camera::config::IO_METHOD_UNKNOWN;
using apollo::drivers::camera::config::IO_METHOD_USERPTR;
using apollo::drivers::camera::config::RGB;
using apollo::drivers::camera::config::YUYV;
using apollo::drivers::camera::config::USB;
using apollo::drivers::camera::config::HIK;

// camera raw image struct
struct CameraImage {
  int width;
  int height;
  int bytes_per_pixel;
  int image_size;
  int is_new;
  int tv_sec;
  int tv_usec;
  char* image;

  ~CameraImage() {
    if (image != nullptr) {
      free(reinterpret_cast<void*>(image));
      image = nullptr;
    }
  }
};

typedef std::shared_ptr<CameraImage> CameraImagePtr;

struct buffer {
  void* start;
  size_t length;
};

class ICam {
public:
  virtual ~ICam() {};

  virtual bool init(const std::shared_ptr<Config>& camera_config) = 0;

  // user use this function to get camera frame data
  virtual bool poll(const CameraImagePtr& raw_image) = 0;

  virtual bool is_capturing() = 0;
  virtual bool wait_for_device(void) = 0;

};
}  // namespace camera
}  // namespace drivers
}  // namespace apollo
