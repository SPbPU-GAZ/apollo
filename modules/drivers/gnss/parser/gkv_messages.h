#pragma once

#include <cstdint>
#include <stddef.h>

namespace apollo {
namespace drivers {
namespace gnss {
namespace gkv {

constexpr size_t CRC_LENGTH = 4;
constexpr uint8_t PACKET_PREAMBLE = 0xFF;

// enum RequestPacketType : uint8_t {
//   CONNECTION_CHECK = 0x00,
//   DEVICE_RESET = 0x01,
//   DEVICE_INFO = 0x04,
//   DEVICE_SETTINGS = 0x06,
//   CUSTOM_PACKET_SETTINGS = 0x26,
//   GET_DATA = 0x17,
//   ANGULAR_VELOCITY_OFFSET_COEFS = 0x1D,
//   GYRO_OFFSET_COEFS = 0x1E,
//   ANGULAR_VELOCITY_OFFSET_COEFS_ACCUMULATION = 0x1C,
//   FILTER_SETTINGS = 0x1F,
//   GET_ALGORITHM_SETTINGS = 0x23,
//   SET_ALGORITHM_SETTINGS = 0x24,
//   GNSS_CORRECTION_MASKING = 0x25,
//   COURSE_CORRECTION = 0x40,
//   ADDITIONAL_INTERFACE_SETTINGS = 0x42
// };

enum ResponsePacketType : uint8_t {
  CONFIRMATION = 0x00,
  DEVICE_INFO = 0x05,
  DEVICE_SETTINGS = 0x07, // read and write
  CUSTOM_PACKET_SETTINGS = 0x27, // read and write
  DATA_ADC_CODES = 0x0A,
  DATA_CALIBRATED_DATA = 0x0B,
  DATA_ORIENTATION = 0x0C,
  DATA_INCLINOMETER = 0x0D,
  DATA_BINS = 0x12,
  DATA_CUSTOM_PACKET = 0x13,
  DATA_GNSS = 0x0E,
  DATA_GNSS_EXTENDED = 0x0F,
  ANGULAR_VELOCITY_OFFSET_COEFS = 0x1E,
  GYRO_OFFSET_COEFS = 0x1D,
  FILTER_SETTINGS = 0x20,
  ALGORITHM_SETTINGS = 0x24
};

#pragma pack(push, 1) // turn off struct padding

struct Header {
  uint8_t preamble;
  uint8_t address;
  uint8_t packet_type;
  uint8_t data_length;
};
static_assert(sizeof(Header) == 4, "Incorrect size of Header");

/// TODO: set real parameters
struct CustomPacket {
  float param1;
  float param2;
};
static_assert(sizeof(CustomPacket) / 4 == 2, "Incorrect size of CustomPacket");

#pragma pack(pop) // back to whatever the previous packing mode was

}  // namespace gkv
}  // namespace gnss
}  // namespace drivers
}  // namespace apollo