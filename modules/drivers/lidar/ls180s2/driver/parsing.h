#ifndef LIDAR_LS180S2_SRC_PARSING_H_
#define LIDAR_LS180S2_SRC_PARSING_H_

#include <cstdint>
#include <array>
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo {
namespace drivers {
namespace ls180s2 {
namespace parsing {

struct DeviceInfo {
  bool is_add_frame_;
  int horizontal_point;
  double g_fAngleAcc_V;
  double prism_angle[4]; // TODO: to std::array ?
};

// enum class EchoMode : uint8_t {
//   SINGLE = 0,
//   DOUBLE = 1
// };

// struct DataPacket {  
//   int64_t counter;
//   EchoMode echo_mode;
//   uint8_t lidar_model; // TODO: to enum ?
//   uint64_t timestamp_s;
//   uint64_t timestamp_ns;
//   std::array<PointXYZIT, 149> points;
//   int start_frame_point_id = -1;
// };

// static const int POINTS_PER_PACKET_SINGLE_ECHO = 1192;
// static const int POINTS_PER_PACKET_DOUBLE_ECHO = 1188;

bool parseDeviceInfoPacket(const uint8_t* const packet, DeviceInfo& result);
// bool parseDataStreamPacket(const uint8_t* const packet, DataPacket& result);

} // namespace parsing
} // namespace ls180s2
} // namespace drivers
} // namespace apollo

#endif // LIDAR_LS180S2_SRC_PARSING_H_