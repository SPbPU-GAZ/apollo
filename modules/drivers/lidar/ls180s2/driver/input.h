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

class InputSocket {
  public:
    static constexpr uint16_t PACKET_SIZE = 1206;
    static constexpr uint16_t MSOP_DATA_PORT_NUMBER = 2368;
    static constexpr uint16_t DIFOP_DATA_PORT_NUMBER = 2369;
    static constexpr int POLL_TIMEOUT_MS = 1000;

  public:
    InputSocket();
    virtual ~InputSocket();
    bool init(uint16_t port, std::string device_ip, std::string group_ip, bool add_multicast = false);
    /** @brief Read one packet.
     *
     * @param packet points to Ls180s2Packet message
     *
     * @returns 0 if successful,
     *          -1 if end of file
     *          > 0 if incomplete packet (is this possible?)
     */
    int getPacket(Ls180s2Packet *packet);

  private:
    bool isInputAvailable(int timeout);

  private:
    int sockfd_;
    in_addr devip_;
    uint16_t port_;
    std::string device_ip_;
};

}  // namespace ls180s2
}  // namespace drivers
}  // namespace apollo

#endif  // LIDAR_LS180S2_SRC_INPUT_H_