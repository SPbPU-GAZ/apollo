#include "hik_cam.h"

#include <fstream>
#include <MvCameraControl.h>

#define MV_CHECK(mv_func, msg) { auto mvRet = mv_func; if (mvRet != MV_OK) { AERROR << msg << " 0x" << std::hex << mvRet << std::dec; return false; } }


apollo::drivers::camera::HikCam::~HikCam()
{
  AINFO << "DEST";

  stop_capturing();
  uninit_device();
  close_device();
}

bool apollo::drivers::camera::HikCam::init(
    const std::shared_ptr<Config>& camera_config) {
  m_config = camera_config;

  AINFO << "INIT";

  // Warning when diff with last > 1.5* interval
  m_frameWarnInterval = static_cast<float>(1.5 / m_config->frame_rate());
  // now max fps 30, we use an appox time 0.9 to drop image.
  m_frameDropInterval = static_cast<float>(0.9 / m_config->frame_rate());

  return true;
}

bool apollo::drivers::camera::HikCam::poll(const CameraImagePtr & raw_image)
{
  AINFO << "POLL";

  raw_image->is_new = 0;
  // free memory in this struct desturctor
  memset(raw_image->image, 0, raw_image->image_size * sizeof(char));

  MV_FRAME_OUT_INFO_EX frInfo;
  MV_CHECK(MV_CC_GetOneFrameTimeout(m_handle, (unsigned char*)raw_image->image, raw_image->image_size * sizeof(char), &frInfo, 1000), "Image acquisition failed:");

  // raw_image->tv_sec = frInfo.nHostTimeStamp;
  // raw_image->tv_usec = frInfo.nHostTimeStamp;

  raw_image->is_new = 1;
  return true;
}

bool apollo::drivers::camera::HikCam::is_capturing()
{
  AINFO << "IS CAPT";

  return m_isGrabbing;
}

bool apollo::drivers::camera::HikCam::wait_for_device(void)
{
  AINFO << "WAIT FOR DEV";

  if (m_isGrabbing) {
    ADEBUG << "is capturing";
    return true;
  }

  if (!open_device())
    return false;
  if (!init_device()) {
    close_device();
    return false;
  }
  if (!start_capturing()) {
    uninit_device();
    close_device();
    return false;
  }

  return true;
}

bool apollo::drivers::camera::HikCam::open_device()
{
  AINFO << "OPEN";

  if (m_handle) {
    AERROR << "Device already opened!";
    return false;
  }

  int devNum = -1;
  try {
    devNum = std::stoi(m_config->camera_dev());
  } catch (...) {
    AERROR << "Invalid camera dev: " << m_config->camera_dev();
    return false;
  }

  MV_CC_DEVICE_INFO_LIST devList;
  MV_CHECK(MV_CC_EnumDevices(MV_USB_DEVICE, &devList), "Device list error:");

  if (devNum < 0 || (uint)devNum >= devList.nDeviceNum) {
    AERROR << "Invalid device number: " << devNum;
    return false;
  }

  MV_CHECK(MV_CC_CreateHandle(&m_handle, devList.pDeviceInfo[devNum]), "Handle creation failed:");

  return true;
}

bool apollo::drivers::camera::HikCam::close_device()
{
  AINFO << "CLOSE";

  if (m_handle)
    MV_CHECK(MV_CC_DestroyHandle(m_handle), "Handle destroy failed:");

  return true;
}

bool apollo::drivers::camera::HikCam::init_device()
{
  AINFO << "INIT DEV";

  MV_CHECK(MV_CC_OpenDevice(m_handle), "Device open failed:");

  // TODO: add image config
  // MV_CHECK(MV_CC_SetEnumValue(m_handle, "PixelFormat", PixelType_Gvsp_RGB8_Planar), "Pixel set:");

  return true;
}

bool apollo::drivers::camera::HikCam::uninit_device()
{
  AINFO << "UNINIT DEV";

  if (m_handle)
    MV_CHECK(MV_CC_CloseDevice(m_handle), "Device close failed:");

  return true;
}

bool apollo::drivers::camera::HikCam::start_capturing()
{
  AINFO << "START CAPT";

  if (is_capturing())
    return true;
  
  MV_CHECK(MV_CC_StartGrabbing(m_handle), "Start grabbing failed:");
  
  m_isGrabbing = true;
  return true;
}

bool apollo::drivers::camera::HikCam::stop_capturing()
{
  AINFO << "STOP CAPT";

  if (is_capturing()) {
    MV_CHECK(MV_CC_StopGrabbing(m_handle), "Stop grabbing failed:");
    m_isGrabbing = false;
  }

  return true;
}
