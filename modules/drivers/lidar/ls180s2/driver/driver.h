#ifndef LIDAR_LS180S2_SRC_DRIVER_H_
#define LIDAR_LS180S2_SRC_DRIVER_H_

#include <unistd.h>
#include <cstdio>
#include <netinet/in.h>
#include <string>
#include <memory>
#include <thread>
#include <regex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/file.h>
#include <chrono>
#include <deque>
#include <mutex>
#include "cyber/time/time.h"
#include "modules/drivers/lidar/proto/ls180s2.pb.h"
#include "modules/drivers/lidar/proto/ls180s2_config.pb.h"
#include "modules/drivers/lidar/ls180s2/driver/input.h"
#include "modules/drivers/lidar/ls180s2/driver/ThreadPool.h"
#include "modules/drivers/lidar/common/driver_factory/driver_base.h"
#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"

namespace apollo {
namespace drivers {
namespace ls180s2 {

static const int POINTS_BYTES_PER_PACKET_SINGLE_ECHO = 1192;
static const int POINTS_BYTES_PER_PACKET_DOUBLE_ECHO = 1188;
// static float g_fDistanceAcc = 0.1 * 0.01f;
// static float m_offset = 6.37f; // TODO: moved to class members
// static double cos30 = cos(DEG2RAD(30));
// static double sin30 = sin(DEG2RAD(30));
// static double cos60 = cos(DEG2RAD(60));
// static double sin60 = sin(DEG2RAD(60));

struct Firing;
struct PointXYZIRT;

class LsLidarDriver final : public lidar::LidarDriver {
  public:
    LsLidarDriver(const std::shared_ptr<apollo::cyber::Node>& node, const Config &config);
    virtual ~LsLidarDriver();
    bool Init() override;

  private:
    void dataPoll();
    void difopPoll();
    bool loadParameters();
    bool createCyberIO();
    void initTimeStamp();
    bool frameRate(const std::shared_ptr<Ls180s2SrvFrameRate>& req, std::shared_ptr<Ls180s2SrvResult>& res);
    bool setDataIp(const std::shared_ptr<Ls180s2SrvDataIp>& req, std::shared_ptr<Ls180s2SrvResult>& res);
    bool setDestinationIp(const std::shared_ptr<Ls180s2SrvDestinationIp>& req, std::shared_ptr<Ls180s2SrvResult>& res);
    bool setDataPort(const std::shared_ptr<Ls180s2SrvDataPort>& req, std::shared_ptr<Ls180s2SrvResult>& res);
    bool setDevPort(const std::shared_ptr<Ls180s2SrvDevPort>& req, std::shared_ptr<Ls180s2SrvResult>& res);
    void setPacketHeader(unsigned char *config_data);
    bool sendPacketTolidar(unsigned char *config_data) const;
    void lslidarChPacketProcess(const std::shared_ptr<Ls180s2Packet> &packet);
    bool isPointInRange(const double &distance) const {
        return (distance >= min_range && distance <= max_range);
    }
    int convertCoordinate(const struct Firing &lidardata);
    void publishPointCloudNew();

 private:
    // Configuration
    Config config_;

    // Socket inputs
    int msop_udp_port{};
    int difop_udp_port{};
    std::shared_ptr<InputSocket> msop_input_;
    std::shared_ptr<InputSocket> difop_input_;

    // Threads
    std::unique_ptr<std::thread> difop_thread_;
    std::unique_ptr<std::thread> polling_thread_;
    std::unique_ptr<ThreadPool> publish_thread_pool_;

    // Ethernet relate variables
    std::string lidar_ip_string;
    std::string group_ip_string;
    std::string frame_id;

    int socket_id;
    bool add_multicast{};
    double prism_angle[4]{};

    // ...
    uint64_t pointcloudTimeStamp{};
    unsigned char packetTimeStamp[10]{};
    unsigned char difop_data[1206]{};
    apollo::cyber::Time timeStamp;

    // Configuration parameters
    double min_range;
    double max_range;
    double packet_rate;

    double packet_end_time;
    double current_packet_time;
    double last_packet_time;

    bool use_time_service;
    int return_mode;
    int scan_start_angle{};
    int scan_end_angle{};
    double g_fAngleAcc_V;
    bool packet_loss;
    bool is_add_frame_;
    bool get_ms06_param;
    bool is_get_difop_;
    std::mutex pc_mutex_;

    std::string pointcloud_channel;
    std::shared_ptr<apollo::cyber::Writer<apollo::drivers::PointCloud>> point_cloud_writer_;

    std::shared_ptr<apollo::cyber::Writer<Ls180s2PacketLoss>> packet_loss_writer_;
    std::shared_ptr<apollo::cyber::Service<Ls180s2SrvFrameRate, Ls180s2SrvResult>> frame_rate_service_;
    std::shared_ptr<apollo::cyber::Service<Ls180s2SrvDataIp, Ls180s2SrvResult>> data_ip_service_;
    std::shared_ptr<apollo::cyber::Service<Ls180s2SrvDestinationIp, Ls180s2SrvResult>> destination_ip_service_;
    std::shared_ptr<apollo::cyber::Service<Ls180s2SrvDataPort, Ls180s2SrvResult>> data_port_service_;
    std::shared_ptr<apollo::cyber::Service<Ls180s2SrvDevPort, Ls180s2SrvResult>> dev_port_service_;

    int64_t last_packet_number_;
    int64_t current_packet_number_;
    int64_t tmp_packet_number_;
    int64_t total_packet_loss_;
    int frame_count;
    int m_horizontal_point = -1;

    // Maths
    double cos_table[36000]{};
    double sin_table[36000]{};
    double cos_mirror_angle[4]{};
    double sin_mirror_angle[4]{};

    // PCL
    pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_;
    pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_bak_;
    pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_pub_;

    // ...
    float m_offset = 6.37f;
    float g_fDistanceAcc = 0.1 * 0.01f;

    std::atomic<uint64_t> sequence_num = {0};
};

}  // namespace ls180s2
}  // namespace drivers
}  // namespace apollo

#endif  // LIDAR_LS180S2_SRC_INPUT_H_