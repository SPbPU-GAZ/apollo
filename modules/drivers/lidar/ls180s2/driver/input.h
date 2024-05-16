#ifndef LIDAR_LS180S2_SRC_INPUT_H_
#define LIDAR_LS180S2_SRC_INPUT_H_

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
#include "modules/drivers/lidar/proto/ls180s2.pb.h"

namespace apollo {
namespace drivers {
namespace ls180s2 {

  static const uint16_t PACKET_SIZE = 1206;
  static const uint16_t MSOP_DATA_PORT_NUMBER = 2368;
  static const uint16_t DIFOP_DATA_PORT_NUMBER = 2369;
  static const int POLL_TIMEOUT = 1000; // one second (in msec)

class Input {
  public:
    Input(uint16_t port, std::string device_ip = "", std::string group_ip = "224.1.1.2", bool add_multicast = false);
    virtual ~Input() {}
    // 0 if successful
    // -1 if end of file
    // >0 if incomplete packet (is this possible?)
    virtual int getPacket(Ls180s2Packet *packet) = 0;
    int getRpm(void);
    int getReturnMode(void);
    bool getUpdateFlag(void);
    void clearUpdateFlag(void);

  protected:
    uint16_t port_;
    std::string device_ip_;
    std::string group_ip_;
    bool add_multicast_;
    int cur_rpm_;
    int return_mode_;
    bool npkt_update_flag_;
};

class InputSocket : public Input {
  public:
    InputSocket(uint16_t port = MSOP_DATA_PORT_NUMBER, std::string device_ip = "", std::string group_ip = "224.1.1.2", bool add_multicast = false);
    virtual ~InputSocket();
    int getPacket(Ls180s2Packet *packet) override;

  private:
    int sockfd_;
    in_addr devip_;
};

}  // namespace ls180s2
}  // namespace drivers
}  // namespace apollo

#endif  // LIDAR_LS180S2_SRC_INPUT_H_