#include "modules/telemetry/output.h"
#include "cyber/common/log.h"
#include "cyber/time/time.h"

namespace apollo {
namespace telemetry {

OutputSocket::OutputSocket() {
  sockfd_ = -1;
  inited_ = false;
}

OutputSocket::~OutputSocket(void) {
  (void)close(sockfd_);
}

bool OutputSocket::init(uint16_t port, std::string device_ip) {
  if (sockfd_ != -1) {
    (void)close(sockfd_);
  }
  
  AINFO << "Opening UDP socket "  << device_ip.c_str() << ":" << port;

  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd_ < 0) { 
    AERROR << "Failed to create socket";
    return false;
  }

  AINFO << "Socket fd = " << sockfd_;

  memset(&server_addr_, 0, sizeof(server_addr_));
  server_addr_.sin_family = AF_INET;
  server_addr_.sin_port = htons(port);
  server_addr_.sin_addr.s_addr = inet_addr(device_ip.c_str());

  inited_ = true;

  return true;
}

int OutputSocket::send_data(const char* data) {
  if (!inited_) {
    AERROR << "Failed to send data to server. Init socket first.";
    return -1;
  }

  auto ret = sendto(sockfd_, data, strlen(data), 0,
                    (const struct sockaddr *) &server_addr_, sizeof(server_addr_));

  if (ret < 0) {
    AERROR << "Failed to send data to server";
    return -1;
  }

  return ret;
}

}  // namespace telemetry
}  // namespace apollo
