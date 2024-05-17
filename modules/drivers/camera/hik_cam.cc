#include "hik_cam.h"

#include <MvCameraControl.h>

#define MV_ERR(mv_func, msg) { auto mvRet = mv_func; if (mvRet != MV_OK) { auto f(std::cout.flags()); AERROR << msg << " 0x" << std::hex << mvRet; std::cout.flags(f); return false; } }

static uint intForMvCeil(uint val, const MVCC_INTVALUE& mvVal)
{
  auto newVal = mvVal.nInc * (val / mvVal.nInc);
  if (val % mvVal.nInc)
    newVal += mvVal.nInc;
  return std::min(std::max(mvVal.nMin, newVal), mvVal.nMax);
}

static uint intForMvFloor(uint val, const MVCC_INTVALUE& mvVal)
{
  auto newVal = mvVal.nInc * (val / mvVal.nInc);
  return std::min(std::max(mvVal.nMin, newVal), mvVal.nMax);
}

static float floatForMV(float val, const MVCC_FLOATVALUE& mvVal)
{
  return std::min(std::max(mvVal.fMin, val), mvVal.fMax);
}

static bool isValueInMvEnum(uint val, const MVCC_ENUMVALUE& en)
{
  for (uint i = 0; i < en.nSupportedNum; i++)
    if (val == en.nSupportValue[i])
      return true;

  return false;
}

template<typename T>
bool setConfigNode(void* handle, const std::string& node, T value)
{
  MV_XML_AccessMode accMode = AM_NI;
  MV_ERR(MV_XML_GetNodeAccessMode(handle, node.c_str(), &accMode), "Node access query failed:");

  if (accMode != AM_WO && accMode != AM_RW) {
    AWARN << "Node " << node << " has not write access!";
    return false;
  }

  MV_XML_InterfaceType ifType = IFT_IBoolean;
  MV_ERR(MV_XML_GetNodeInterfaceType(handle, node.c_str(), &ifType), "Node type query failed:");

  switch (ifType)
  {
  case IFT_IBoolean:
    MV_ERR(MV_CC_SetBoolValue(handle, node.c_str(), value), "Boolean node range query failed:");
    break;

  case IFT_IInteger:
    {
      MVCC_INTVALUE valRange;
      MV_ERR(MV_CC_GetIntValue(handle, node.c_str(), &valRange), "Int node range query failed:");

      auto newVal = intForMvCeil(value, valRange);
      if (newVal != value)
        AWARN << "Changed " << node << " value from " << value << " to " << newVal;

      MV_ERR(MV_CC_SetIntValue(handle, node.c_str(), newVal), "Int node value set failed:");
    }
    break;

  case IFT_IFloat:
    {
      MVCC_FLOATVALUE valRange;
      MV_ERR(MV_CC_GetFloatValue(handle, node.c_str(), &valRange), "Float node range query failed:");

      auto newVal = floatForMV(value, valRange);
      if (std::abs(value - newVal) >= std::numeric_limits<float>::epsilon())
        AWARN << "Changed " << node << " value from " << value << " to " << newVal;

      MV_ERR(MV_CC_SetFloatValue(handle, node.c_str(), newVal), "Float node value set failed:");
    }
    break;

  case IFT_IEnumeration:
    MV_ERR(MV_CC_SetEnumValue(handle, node.c_str(), value), "Enum node value set failed:");
    break;
  
  default:
    AWARN << "Unexpected node type: " << ifType;
    return false;
  }

  return true;
}


apollo::drivers::camera::HikCam::~HikCam()
{
  stop_capturing();
  uninit_device();
  close_device();
}

bool apollo::drivers::camera::HikCam::init(
    const std::shared_ptr<Config>& camera_config) {
  m_config = camera_config;

  return true;
}

bool apollo::drivers::camera::HikCam::poll(const CameraImagePtr & raw_image)
{
  raw_image->is_new = 0;
  // free memory in this struct desturctor
  memset(raw_image->image, 0, raw_image->image_size * sizeof(char));

  if (!m_acqTimeoutMs) {
    AERROR << "Invalid frame acquisition timeout!";
    return false;
  }

  MV_FRAME_OUT frame;
  MV_ERR(MV_CC_GetImageBuffer(m_handle, &frame, m_acqTimeoutMs), "Image buffer acquisition failed:");

  const MV_FRAME_OUT_INFO_EX& frameInfo = frame.stFrameInfo;
  uint8_t* src = frame.pBufAddr;
  uint8_t* dst = (uint8_t*)raw_image->image;

  const auto rowsCnt = frameInfo.nExtendHeight - m_hDeltaPx - m_yDeltaPx;
  const auto rowSizePx = frameInfo.nExtendWidth - m_xDeltaPx - m_wDeltaPx;

  if (uint(raw_image->image_size) != (m_pixelSizeBytes * rowsCnt * rowSizePx)) {
    AERROR << "Image array size mismatch: " << raw_image->image_size <<
      " != " << m_pixelSizeBytes * rowsCnt * rowSizePx;

      MV_ERR(MV_CC_FreeImageBuffer(m_handle, &frame), "Image buffer dealloc failed:");
      return false;
  }

  {
    const auto preSrcDelta = m_pixelSizeBytes * m_xDeltaPx;
    const auto cpySize = m_pixelSizeBytes * rowSizePx;
    const auto postSrcDelta = m_pixelSizeBytes * (m_wDeltaPx + rowSizePx);

    src += m_pixelSizeBytes * m_yDeltaPx * frameInfo.nExtendWidth;

    for (uint i = 0; i < rowsCnt; i++) {
      src += preSrcDelta;

      std::memcpy(dst, src, cpySize);
      dst += cpySize;

      src += postSrcDelta;
    }
  }

  int64_t frameTsMs = frameInfo.nHostTimeStamp;
  if (m_config->hardware_trigger()) {
    frameTsMs = frameInfo.nDevTimeStampHigh;
    frameTsMs = (frameTsMs << 32) + frameInfo.nDevTimeStampLow;
  }

  raw_image->tv_sec = (int)(frameTsMs / 1000);
  raw_image->tv_usec = (int)(frameTsMs * 1000);

  MV_ERR(MV_CC_FreeImageBuffer(m_handle, &frame), "Image buffer dealloc failed:");

  raw_image->is_new = 1;
  return true;
}

bool apollo::drivers::camera::HikCam::is_capturing()
{
  MVCC_INTVALUE uptime;
  auto uptimeRes = MV_CC_GetIntValue(m_handle, "DeviceUptime", &uptime) == MV_OK;

  return uptimeRes && m_isGrabbing;
}

bool apollo::drivers::camera::HikCam::wait_for_device(void)
{
  if (is_capturing())
    return true;

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
  if (m_handle) {
    stop_capturing();
    uninit_device();
    close_device();
  }

  int devNum = -1;
  try {
    devNum = std::stoi(m_config->camera_dev());
  } catch (...) {
    AERROR << "Invalid camera dev: " << m_config->camera_dev();
    return false;
  }

  MV_CC_DEVICE_INFO_LIST devList;
  MV_ERR(MV_CC_EnumDevices(MV_USB_DEVICE, &devList), "Device list error:");

  if (devNum < 0 || (uint)devNum >= devList.nDeviceNum) {
    AERROR << "Invalid device number: " << devNum;
    return false;
  }

  MV_ERR(MV_CC_CreateHandle(&m_handle, devList.pDeviceInfo[devNum]), "Handle creation failed:");

  return true;
}

bool apollo::drivers::camera::HikCam::close_device()
{
  if (m_handle) {
    MV_ERR(MV_CC_DestroyHandle(m_handle), "Handle destroy failed:");
    m_handle = nullptr;
  }

  return true;
}

bool apollo::drivers::camera::HikCam::init_device()
{
  MV_ERR(MV_CC_OpenDevice(m_handle), "Device open failed:");

  return set_device_config();
}

bool apollo::drivers::camera::HikCam::uninit_device()
{
  if (m_handle) {
    MV_ERR(MV_CC_CloseDevice(m_handle), "Device close failed:");
    m_isGrabbing = false;
  }

  reset_roi();

  m_pixelSizeBytes = 0;
  m_acqTimeoutMs = 0;

  return true;
}

bool apollo::drivers::camera::HikCam::start_capturing()
{
  if (is_capturing())
    return true;
  
  MV_ERR(MV_CC_StartGrabbing(m_handle), "Start grabbing failed:");
  
  m_isGrabbing = true;
  return true;
}

bool apollo::drivers::camera::HikCam::stop_capturing()
{
  if (is_capturing()) {
    MV_ERR(MV_CC_StopGrabbing(m_handle), "Stop grabbing failed:");
    m_isGrabbing = false;
  }

  return true;
}

static float redFromKelvin(float k)
{
  k /= 100.f;

  if (k <= 66.f)
    return 255.f;

  auto r = k - 60.f;
  r = 329.698727446f * std::pow(r, -0.1332047592f);
  return std::min(std::max(0.f, r), 255.f);
}

static float greenFromKelvin(float k)
{
  k /= 100.f;

  auto g = 0.f;
  if (k <= 66.f)
    g = 99.4708025861f * std::log(k) - 161.1195681661f;
  else {
    g = k - 60.f;
    g = 288.1221695283f * std::log(g) -0.0755148492f;
  }

  return std::min(std::max(0.f, g), 255.f);
}

static float blueFromKelvin(float k)
{
  k /= 100.f;

  if (k >= 66.f)
    return 255.f;

  if (k <= 19.f)
    return 0.f;

  auto b = k - 10.f;
  b = 138.5177312231f * std::log(b) - 305.0447927307f;
  return std::min(std::max(0.f, b), 255.f);
}

bool apollo::drivers::camera::HikCam::set_device_config()
{
  auto pixelType = PixelType_Gvsp_Undefined;
  switch(m_config->output_type())
  {
    case RGB:
      pixelType = PixelType_Gvsp_RGB8_Packed;
      m_pixelSizeBytes = 3;
      break;

    case YUYV:
      pixelType = PixelType_Gvsp_YUV422_YUYV_Packed;
      m_pixelSizeBytes = 2;
      break;

    default:
      AWARN << "Unexpected output type: " << m_config->output_type();
      break;
  }
  setConfigNode<MvGvspPixelType>(m_handle, "PixelFormat", pixelType);

  if (!(set_binning() && set_roi()))
    reset_roi();

  setConfigNode<bool>(m_handle, "AcquisitionFrameRateEnable", true);
  setConfigNode<MV_CAM_ACQUISITION_MODE>(m_handle, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
  setConfigNode<float>(m_handle, "AcquisitionFrameRate", float(m_config->frame_rate()));

  MVCC_FLOATVALUE resFrameRate;
  MV_ERR(MV_CC_GetFloatValue(m_handle, "ResultingFrameRate", &resFrameRate), "Resulting frame rate query failed:");
  AINFO << "Resulting frame rate is: " << resFrameRate.fCurValue;

  m_acqTimeoutMs = uint(1000 / resFrameRate.fCurValue);

  if (m_config->brightness() != -1)
    setConfigNode<uint>(m_handle, "Brightness", m_config->brightness());

  if (m_config->contrast() != -1)
    setConfigNode<uint>(m_handle, "Contrast", m_config->contrast());

  if (m_config->saturation() == -1)
    setConfigNode<bool>(m_handle, "SaturationEnable", false);
  else {
    setConfigNode<bool>(m_handle, "SaturationEnable", true);

    setConfigNode<uint>(m_handle, "SaturationAuto", 0);
    setConfigNode<uint>(m_handle, "Saturation", m_config->saturation());
  }

  if (m_config->sharpness() == -1)
    setConfigNode<bool>(m_handle, "SharpnessEnable", false);
  else {
    setConfigNode<bool>(m_handle, "SharpnessEnable", true);

    setConfigNode<uint>(m_handle, "SharpnessAuto", 0);
    setConfigNode<uint>(m_handle, "Sharpness", m_config->sharpness());
  }

  if (m_config->gain() == -1)
    setConfigNode<MV_CAM_GAIN_MODE>(m_handle, "GainAuto", MV_GAIN_MODE_CONTINUOUS);
  else {
    setConfigNode<MV_CAM_GAIN_MODE>(m_handle, "GainAuto", MV_GAIN_MODE_OFF);
    setConfigNode<float>(m_handle, "Gain", float(m_config->gain()));
  }

  if (m_config->auto_exposure())
    setConfigNode<MV_CAM_EXPOSURE_AUTO_MODE>(m_handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_CONTINUOUS);
  else {
    setConfigNode<MV_CAM_EXPOSURE_AUTO_MODE>(m_handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    setConfigNode<MV_CAM_EXPOSURE_MODE>(m_handle, "ExposureMode", MV_EXPOSURE_MODE_TIMED);
    setConfigNode<float>(m_handle, "ExposureTime", float(m_config->exposure()));
  }

  if (m_config->auto_white_balance())
    setConfigNode<MV_CAM_BALANCEWHITE_AUTO>(m_handle, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  else {
    setConfigNode<MV_CAM_BALANCEWHITE_AUTO>(m_handle, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_OFF);

    float red = redFromKelvin(m_config->white_balance());
    float green = greenFromKelvin(m_config->white_balance());
    float blue = blueFromKelvin(m_config->white_balance());

    uint rRat = STANDARD_WB_VAL * red / green;
    uint bRat = STANDARD_WB_VAL * blue / green;
    
    setConfigNode<uint>(m_handle, "BalanceRatioSelector", 0);
    setConfigNode<uint>(m_handle, "BalanceRatio", rRat);

    setConfigNode<uint>(m_handle, "BalanceRatioSelector", 1);
    setConfigNode<uint>(m_handle, "BalanceRatio", STANDARD_WB_VAL);

    setConfigNode<uint>(m_handle, "BalanceRatioSelector", 2);
    setConfigNode<uint>(m_handle, "BalanceRatio", bRat);
  }

  return true;
}

bool apollo::drivers::camera::HikCam::set_binning()
{
  if (!setConfigNode<uint>(m_handle, "BinningSelector", 0)) {
    AERROR << "Binning region selection failed!";
    return false;
  }

  MVCC_ENUMVALUE binnVals;

  MV_ERR(MV_CC_GetEnumValue(m_handle, "BinningHorizontal", &binnVals), "Hor binning query failed:");
  if (!isValueInMvEnum(m_config->binning_hor(), binnVals)) {
    AERROR << "Horizontal binning value is not supported!";
    return false;
  }

  MV_ERR(MV_CC_GetEnumValue(m_handle, "BinningVertical", &binnVals), "Ver binning query failed:");
  if (!isValueInMvEnum(m_config->binning_ver(), binnVals)) {
    AERROR << "Vertical binning value is not supported!";
    return false;
  }

  setConfigNode<uint>(m_handle, "BinningHorizontal", m_config->binning_hor());
  setConfigNode<uint>(m_handle, "BinningVertical", m_config->binning_ver());

  return true;
}

bool apollo::drivers::camera::HikCam::set_roi()
{
  if (!setConfigNode<uint>(m_handle, "FrameSpecInfoSelector", 8) ||
    !setConfigNode<bool>(m_handle, "FrameSpecInfo", true)) {
    AERROR << "ROI position info enabling failed!";
    return false;
  }

  MVCC_INTVALUE valRange;

  MV_ERR(MV_CC_GetIntValue(m_handle, "WidthMax", &valRange), "Max width query failed:");
  auto widthMax = valRange.nCurValue;

  MV_ERR(MV_CC_GetIntValue(m_handle, "HeightMax", &valRange), "Max height query failed:");
  auto heightMax = valRange.nCurValue;

  if ((m_config->offset_x() + m_config->width()) > widthMax ||
    (m_config->offset_y() + m_config->height()) > heightMax) {
      AERROR << "Invalid image size and offset values: w(" << (m_config->offset_x() + m_config->width()) <<
        " and " << widthMax << "); h(" << (m_config->offset_y() + m_config->height()) << " and " << heightMax << ")";
      return false;
  }

  MV_ERR(MV_CC_GetIntValue(m_handle, "OffsetX", &valRange), "OffsetX range query failed:");
  // Max value may be limited by old width value
  valRange.nMax = widthMax;
  auto xOffset = intForMvFloor(m_config->offset_x(), valRange);
  m_xDeltaPx = m_config->offset_x() - xOffset;

  MV_ERR(MV_CC_GetIntValue(m_handle, "OffsetY", &valRange), "OffsetY range query failed:");
  // Max value may be limited by old height value
  valRange.nMax = heightMax;
  auto yOffset = intForMvFloor(m_config->offset_y(), valRange);
  m_yDeltaPx = m_config->offset_y() - yOffset;

  // Resetting offset to not limit new width and height
  setConfigNode<uint>(m_handle, "OffsetX", 0);
  setConfigNode<uint>(m_handle, "OffsetY", 0);

  MV_ERR(MV_CC_GetIntValue(m_handle, "Width", &valRange), "Width range query failed:");
  auto width = intForMvCeil(m_config->width() + m_xDeltaPx, valRange);
  MV_ERR(MV_CC_SetIntValue(m_handle, "Width", width), "Width set failed:");
  m_wDeltaPx = width - (m_config->width() + m_xDeltaPx);

  MV_ERR(MV_CC_GetIntValue(m_handle, "Height", &valRange), "Height range query failed:");
  auto height = intForMvCeil(m_config->height() + m_yDeltaPx, valRange);
  MV_ERR(MV_CC_SetIntValue(m_handle, "Height", height), "Height set failed:");
  m_hDeltaPx = height - (m_config->height() + m_yDeltaPx);

  MV_ERR(MV_CC_SetIntValue(m_handle, "OffsetX", xOffset), "X offset set failed:");
  MV_ERR(MV_CC_SetIntValue(m_handle, "OffsetY", yOffset), "Y offset set failed:");

  return true;
}

void apollo::drivers::camera::HikCam::reset_roi()
{
  m_xDeltaPx = 0;
  m_yDeltaPx = 0;
  m_wDeltaPx = 0;
  m_hDeltaPx = 0;
}
