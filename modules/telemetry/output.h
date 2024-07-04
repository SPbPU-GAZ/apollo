#ifndef TELEMETRY_SRC_INPUT_H_
#define TELEMETRY_SRC_INPUT_H_

#include <unistd.h>
#include <cstdio>
#include <netinet/in.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/file.h>
#include <csignal>

namespace apollo {
namespace telemetry {

class OutputSocket {
  public:
    OutputSocket();
    virtual ~OutputSocket();
    bool init(uint16_t port, std::string device_ip);
    // -1 if failed, otherwise - number of bytes sent
    int send_data(const char* data);

  private:
    bool inited_;
    int sockfd_;
    sockaddr_in server_addr_;
};

}  // namespace telemery
}  // namespace apollo

#endif  // TELEMETRY_SRC_INPUT_H_