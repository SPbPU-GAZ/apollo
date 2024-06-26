#include "modules/drivers/gnss/util/gnss_conversion.h"

#include <ctime>
#include <cstdio>

namespace apollo {
namespace drivers {
namespace util {

std::tm* gmtime_wrap(const std::time_t *timer, std::tm *buf) {
#if defined(_WIN32)
    gmtime_s(buf, timer);
    return buf;
#else
    return gmtime_r(timer, buf);
#endif
}

int time_ns_to_str_ddmmyy(uint64_t time_ns, char* buffer, size_t bufferSize) {
  std::tm tm_struct{};
  const std::time_t timeSec = time_ns / 1000000000UL;
  gmtime_wrap(&timeSec, &tm_struct);
  
  char formatBuffer[32] = "%d%m%y";

  return (int)std::strftime(buffer, bufferSize, formatBuffer, &tm_struct);
}

int time_ns_to_str_hhmmss_msms(uint64_t time_ns, char* buffer, size_t bufferSize) {
  int offset = 0, tmp;
  
  std::tm tm_struct{};
  const std::time_t timeSec = time_ns / 1000000000UL;
  gmtime_wrap(&timeSec, &tm_struct);
  
  char formatBuffer[32] = "%H%M%S.";
  tmp = (int)std::strftime(buffer, bufferSize, formatBuffer, &tm_struct);
  if (tmp == 0) return 0;
  offset += tmp;
  
  const auto part_ns = time_ns % 1000000000UL;
  tmp = snprintf(buffer + offset, bufferSize - offset, "%d", int(part_ns / 10000000));
  if (tmp <= 0) return 0;
  offset += tmp;

  return tmp;
}

int lat_deg_to_str_ddmm_mmmm(double lat_deg, char* buffer, size_t bufferSize) {
  const int dd = int(lat_deg);
  const double mins = (lat_deg - dd) * 60.0;
  const int mm = int(mins);
  const int mmmm = int(10000 * (mins - mm));

  int tmp = snprintf(buffer, bufferSize, "%02d%02d.%04d", dd, mm, mmmm);
  if (tmp <= 0) return 0;

  return tmp;
}

int lon_deg_to_str_dddmm_mmmm(double lon_deg, char* buffer, size_t bufferSize) {
  const int ddd = int(lon_deg);
  const double mins = (lon_deg - ddd) * 60.0;
  const int mm = int(mins);
  const int mmmm = int(10000 * (mins - mm));

  int tmp = snprintf(buffer, bufferSize, "%03d%02d.%04d", ddd, mm, mmmm);
  if (tmp <= 0) return 0;

  return tmp;
}

}  // namespace util
}  // namespace drivers
}  // namespace apollo