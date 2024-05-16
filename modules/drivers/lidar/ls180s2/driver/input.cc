#include "modules/drivers/lidar/ls180s2/driver/input.h"
#include "cyber/common/log.h"
#include "cyber/time/time.h"

namespace apollo {
namespace drivers {
namespace ls180s2 {

Input::Input(uint16_t port, std::string device_ip, std::string group_ip, bool add_multicast) {
  port_ = port;
  device_ip_ = device_ip;
  group_ip_ = group_ip;
  add_multicast_ = add_multicast;
  npkt_update_flag_ = false;
  cur_rpm_ = 0;
  return_mode_ = 1;
}
  
int Input::getRpm(void) {
  return cur_rpm_;
}
  
int Input::getReturnMode(void) {
  return return_mode_;
 }
  
bool Input::getUpdateFlag(void) {
  return npkt_update_flag_;
}
  
void Input::clearUpdateFlag(void) {
  npkt_update_flag_ = false;
}

InputSocket::InputSocket(uint16_t port, std::string device_ip, std::string group_ip, bool add_multicast) : Input(port, device_ip, group_ip, add_multicast) {
  sockfd_ = -1;

  if (!device_ip.empty()) {
    inet_aton(device_ip.c_str(), &devip_);
  }

  AINFO << "Opening UDP socket port: "  << port;
  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ == -1) {
    AERROR << "Failed to create socket (socket)";
    return;
  }

  int opt = 1;
  if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
      perror("setsockopt error!\n");
      AERROR << "Failed to set socket option (setsockopt)";
      return;
  }

  sockaddr_in my_addr{};                        // my address information
  memset(&my_addr, 0, sizeof(my_addr));         // initialize to zeros
  my_addr.sin_family = AF_INET;                 // host byte order
  my_addr.sin_port = htons(port);               // port in network byte order
  my_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // automatically fill in my IP

  if (bind(sockfd_, (sockaddr *) &my_addr, sizeof(sockaddr)) == -1) {
      AERROR << "Failed to bind socket to address (bind)";
      return;
  }

  if (add_multicast) {
    struct ip_mreq group{};
    group.imr_multiaddr.s_addr = inet_addr(group_ip.c_str());
    group.imr_interface.s_addr = htonl(INADDR_ANY);

    if (setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &group, sizeof(group)) < 0) {
      AERROR << "Failed to add multicast group (setsockopt)";
      close(sockfd_);
      exit(1);
    }
    else {
      AINFO << "Adding multicast group...OK";
    }
  }

  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    AERROR << "Failed to set non-block socket (fcntl)";
    return;
  }
}

InputSocket::~InputSocket(void) {
  (void) close(sockfd_);
}

int InputSocket::getPacket(Ls180s2Packet *packet) {
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;

  sockaddr_in sender_address{};
  socklen_t sender_address_len = sizeof(sender_address);

  double time1 = apollo::cyber::Time().Now().ToSecond();
  while (true) {
    do {
      int retval = poll(fds, 1, POLL_TIMEOUT);

    /// TODO: see velodyne/driver/socket_input.cc
    //   if (retval < 0) { // poll() error?
    //     if (errno != EINTR) {
    //         AWARN << "lslidar port " << port_ << "poll() error: " << strerror(errno);
    //     }
    //     return false;
    //   }

      if (retval == 0) {
        AWARN << "lslidar poll() timeout. port: " << port_;
        return 1;
      }

    /// TODO: see velodyne/driver/socket_input.cc
    // if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
    //     (fds[0].revents & POLLNVAL)) {  // device error?
    //   AERROR << "lslidar port " << port_ << "poll() reports error";
    //   return false;
    // }
    }
    while ((fds[0].revents & POLLIN) == 0);

    // receive packets that should now be available from the socket using a blocking read
    uint8_t bytes[PACKET_SIZE];
    ssize_t nbytes = recvfrom(sockfd_, bytes, PACKET_SIZE, 0, (sockaddr *) &sender_address, &sender_address_len);

    /// TODO: see velodyne/driver/socket_input.cc
    // if (nbytes < 0) {
    //   if (errno != EWOULDBLOCK) {
    //     AERROR << "recvfrom fail from port " << port_;
    //     return 1;
    //   }
    // }

    if ((size_t) nbytes == PACKET_SIZE) {
      if (device_ip_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr) {
        AWARN_EVERY(100) << "lidar ip parameter error, please reset lidar ip in launch file";
        continue;
      }
      else {
        // read successful, done now
        packet->set_data(bytes, PACKET_SIZE);
        break;
      }
    }

    AERROR << "Incomplete ls180s2 rising data packet read: " << nbytes << " bytes from port " << port_;
  }
  double time2 = apollo::cyber::Time().Now().ToSecond();
  packet->set_stamp(apollo::cyber::Time((time2 + time1) / 2.0).ToNanosecond());

  return 0;
}

}  // namespace ls180s2
}  // namespace drivers
}  // namespace apollo
