#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/time/time.h"

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

#include <gflags/gflags.h>

DEFINE_uint32(server_port, 9999, "Server ipv4 port to listent UDP data");
DEFINE_int32(show_max_symbols, -1, "Max symbols to print in console. -1 to skip truncating.");

namespace {

class InputSocket {
  public:
    static constexpr size_t BUFFER_SIZE = 1024 * 1024;
    static constexpr int POLL_TIMEOUT_MS = 5000;

  public:
    InputSocket() {
      sockfd_ = -1;
      port_ = 0;
      device_ip_ = "";
    }

    ~InputSocket() {
      (void)close(sockfd_);
    }

    bool init(uint16_t port, std::string device_ip, std::string group_ip, bool add_multicast = false) {
      if (sockfd_ != -1) {
        (void)close(sockfd_);
      }

      device_ip_ = device_ip;
      if (!device_ip.empty()) {
        inet_aton(device_ip.c_str(), &devip_);
      }

      AINFO << "Opening UDP socket port: "  << port;
      port_ = port;
      sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
      if (sockfd_ == -1) {
        AERROR << "Failed to create socket (socket)";
        return false;
      }

      int opt = 1;
      if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
          perror("setsockopt error!\n");
          AERROR << "Failed to set socket option (setsockopt)";
          return false;
      }

      sockaddr_in my_addr;                          // my address information
      memset(&my_addr, 0, sizeof(my_addr));         // initialize to zeros
      my_addr.sin_family = AF_INET;                 // host byte order
      my_addr.sin_port = htons(port);               // port in network byte order
      my_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // automatically fill in my IP

      if (bind(sockfd_, (sockaddr *) &my_addr, sizeof(sockaddr)) == -1) {
          AERROR << "Failed to bind socket to address (bind)";
          return false;
      }

      if (add_multicast) {
        struct ip_mreq group{};
        group.imr_multiaddr.s_addr = inet_addr(group_ip.c_str());
        group.imr_interface.s_addr = htonl(INADDR_ANY);

        if (setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &group, sizeof(group)) < 0) {
          AERROR << "Failed to add multicast group (setsockopt)";
          close(sockfd_);
          return false;
        }
        else {
          AINFO << "Adding multicast group...OK";
        }
      }

      if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
        AERROR << "Failed to set non-block mode socket (fcntl)";
        return false;
      }

      AINFO << "Socket fd = " << sockfd_ << ", port = " << port_;
      return true;
    }

    /** @brief Read one packet.
     * @returns 0 if successful,
     *          -1 if end of file
     *          > 0 if incomplete packet (is this possible?)
     */
    int getPacket(std::string& packet) {
      sockaddr_in sender_address{};
      socklen_t sender_address_len = sizeof(sender_address);

      while (true) {
        if (!isInputAvailable(POLL_TIMEOUT_MS)) {
          return 1;
        }

        uint8_t bytes[BUFFER_SIZE];
        ssize_t nbytes = recvfrom(sockfd_, bytes, BUFFER_SIZE, 0, (sockaddr *) &sender_address, &sender_address_len);

        if (nbytes < 0) {
          if (errno != EWOULDBLOCK) {
            AERROR << "recvfrom fail from port " << port_;
            return 1;
          }
        }
        else {
          if (device_ip_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr) {
            AWARN_EVERY(100) << "ip parameter incorrect, please reset ip";
            continue;
          }
          else {
            AINFO << "read " << nbytes << " bytes!";
            packet = std::string((const char *) bytes, nbytes);
            break;
          }
        }

        AERROR << "Incomplete rising data packet read: " << nbytes << " bytes from port " << port_;
      }

      return 0;
    }

  private:
    bool isInputAvailable(int timeout) {
      struct pollfd fds[1];
      fds[0].fd = sockfd_;
      fds[0].events = POLLIN;

      do {
        int retval = poll(fds, 1, timeout);

        if (retval < 0) {
          if (errno != EINTR) {
              AWARN << "port " << port_ << "poll() error: " << strerror(errno);
          }
          return false;
        }

        if (retval == 0) {
          AWARN << "poll() timeout. port: " << port_;
          return false;
        }

        if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL)) {
          AERROR << "port " << port_ << "poll() reports error";
          return false;
        }
      }
      while ((fds[0].revents & POLLIN) == 0);

      return true;
    }

  private:
    int sockfd_;
    in_addr devip_;
    uint16_t port_;
    std::string device_ip_;
};
}

int main(int argc, char **argv) {
  apollo::cyber::Init("planning_pad_terminal");
  FLAGS_alsologtostderr = true;
  FLAGS_v = 3;
  google::ParseCommandLineFlags(&argc, &argv, true);

  InputSocket input_sock;
  if (!input_sock.init(FLAGS_server_port, "", "", false)) {
    AERROR << "Failed to init input socket";
    apollo::cyber::AsyncShutdown();
    return 0;
  }

  // polling
  while (!apollo::cyber::IsShutdown()) {
    std::string packet;
    if (input_sock.getPacket(packet) == 0) {
      if (FLAGS_show_max_symbols > 0 && packet.size() > (size_t)FLAGS_show_max_symbols) {
        packet.resize(FLAGS_show_max_symbols);
      }

      AINFO << packet.c_str();
    }
  }

  apollo::cyber::WaitForShutdown();
  return 0;
}
