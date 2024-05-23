#include "modules/drivers/lidar/ls180s2/driver/parsing.h"
#include <math.h>

namespace apollo {
namespace drivers {
namespace ls180s2 {
namespace parsing {

bool parseDeviceInfoPacket(const uint8_t* const packet, DeviceInfo& result) {
  // check input
  if (!packet) {
    return false;
  }

  // check packet identification header
  bool packet_valid = ((packet[0] == 0x00 || packet[0] == 0xa5) &&
                        packet[1] == 0xff && packet[2] == 0x00 && packet[3] == 0x5a);
  if (!packet_valid) {
    return false;
  }

  // parse info
  if (packet[231] == 64 || packet[231] == 65) {
    result.is_add_frame_ = true;
  }

  result.horizontal_point = packet[184] * 256 + packet[185];
  const int majorVersion = packet[1202];
  const int minorVersion1 = packet[1203] / 16;

  if (1 > majorVersion || (1 == majorVersion && minorVersion1 > 1)) {
    result.g_fAngleAcc_V = 0.0025;
  }
  else {
    result.g_fAngleAcc_V = 0.01;
  }

  float fInitAngle_V = packet[188] * 256 + packet[189];
  if (fInitAngle_V > 32767) {
    fInitAngle_V = fInitAngle_V - 65536;
  }
  result.prism_angle[0] = fInitAngle_V * result.g_fAngleAcc_V;

  fInitAngle_V = packet[190] * 256 + packet[191];
  if (fInitAngle_V > 32767) {
      fInitAngle_V = fInitAngle_V - 65536;
  }
  result.prism_angle[1] = fInitAngle_V * result.g_fAngleAcc_V;

  fInitAngle_V = packet[192] * 256 + packet[193];
  if (fInitAngle_V > 32767) {
    fInitAngle_V = fInitAngle_V - 65536;
  }
  result.prism_angle[2] = fInitAngle_V * result.g_fAngleAcc_V;

  fInitAngle_V = packet[194] * 256 + packet[195];
  if (fInitAngle_V > 32767) {
    fInitAngle_V = fInitAngle_V - 65536;
  }
  result.prism_angle[3] = fInitAngle_V * result.g_fAngleAcc_V;

  return true;
}

/*
bool parseDataStreamPacket(const uint8_t* const packet, DataPacket& result) {
  // check input
  if (!packet) {
    return false;
  }

  // echo mode
  result.echo_mode = (packet[1205] == 0x02) ? EchoMode::DOUBLE : EchoMode::SINGLE;

  // lidar model
  result.lidar_model = packet[1204];

  // packet counter
  result.counter = (packet[1192] << 8) + packet[1193];

  // parse time
  if (packet[1194] == 0xff) {
      result.timestamp_s = (packet[1195] * 0) +
                           (packet[1196] << 24) +
                           (packet[1197] << 16) +
                           (packet[1198] << 8) +
                           (packet[1199] * pow(2, 0));

      result.timestamp_ns = (packet[1200] << 24) +
                            (packet[1201] << 16) +
                            (packet[1202] << 8) +
                            (packet[1203]);
  }
  else {
      struct tm cur_time{};
      memset(&cur_time, 0, sizeof(cur_time));
      cur_time.tm_sec = packet[1199];
      cur_time.tm_min = packet[1198];
      cur_time.tm_hour = packet[1197];
      cur_time.tm_mday = packet[1196];
      cur_time.tm_mon = (int)packet[1195] - 1;
      cur_time.tm_year = (int)packet[1194] + 2000 - 1900;
      result.timestamp_s = static_cast<uint64_t>(timegm(&cur_time));

      result.timestamp_ns = packet[1203] +
                            (packet[1202] << 8) +
                            (packet[1201] << 16) +
                            (packet[1200] << 24);
  }

  // parse points
  if (result.echo_mode == EchoMode::SINGLE) {
    for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET_SINGLE_ECHO; point_idx += 8) {
      // is point frame start mark
      if ((packet[point_idx] == 0xff) &&
          (packet[point_idx + 1] == 0xaa) &&
          (packet[point_idx + 2] == 0xbb) &&
          (packet[point_idx + 3] == 0xcc) &&
          (packet[point_idx + 4] == 0xdd)) {
        result.start_frame_point_id = point_idx;
      }
      // regular point
      else {
        /// TODO: implement
      }
    }
  }
  else {
    /// TODO: implement
  }
}
*/

}  // namespace parsing
}  // namespace ls180s2
}  // namespace drivers
}  // namespace apollo