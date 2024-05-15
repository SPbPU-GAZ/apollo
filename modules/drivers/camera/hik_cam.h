#pragma once

#include "usb_cam.h"

namespace apollo {
namespace drivers {
namespace camera {

class HikCam {
public:
  static constexpr uint STANDARD_WB_VAL = 1024u;

public:
  virtual ~HikCam();

  virtual bool init(const std::shared_ptr<Config>& camera_config);
  // user use this function to get camera frame data
  virtual bool poll(const CameraImagePtr& raw_image);

  bool is_capturing();
  bool wait_for_device(void);

protected:
  bool open_device();
  bool close_device();

  bool init_device();
  bool uninit_device();

  bool start_capturing();
  bool stop_capturing();

  bool set_device_config();

protected:
  std::shared_ptr<Config> m_config;

  void* m_handle = nullptr;
  bool m_isGrabbing = false;

  uint m_acqTimeout = 0;
};

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
