#pragma once

#include <stdint.h>
#include <ctime>

// #include "modules/drivers/gnss/util/macros.h"

namespace apollo {
namespace drivers {
namespace util {

std::tm* gmtime_wrap(const std::time_t *timer, std::tm *buf);

int time_ns_to_str_ddmmyy(uint64_t time_ns, char* buffer, size_t bufferSize);

int time_ns_to_str_hhmmss_msms(uint64_t time_ns, char* buffer, size_t bufferSize);

int lat_deg_to_str_ddmm_mmmm(double lat_deg, char* buffer, size_t bufferSize);

int lon_deg_to_str_dddmm_mmmm(double lon_deg, char* buffer, size_t bufferSize);

}  // namespace util
}  // namespace drivers
}  // namespace apollo
