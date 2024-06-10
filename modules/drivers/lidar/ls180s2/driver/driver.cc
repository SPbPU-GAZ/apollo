#include "modules/drivers/lidar/ls180s2/driver/driver.h"
#include "cyber/common/log.h"

namespace apollo {
namespace drivers {
namespace ls180s2 {

struct Firing {
  double vertical_angle;
  double azimuth;
  double distance;
  float intensity;
  double time;
  int channel_number;
};

struct PointXYZIRT {
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  uint16_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;

LsLidarDriver::LsLidarDriver(const std::shared_ptr<::apollo::cyber::Node>& node, const Config &config) :
  LidarDriver(node) {
  // initialize variables
  config_ = config;
  socket_id = -1;
  min_range = 0.15;
  max_range = 200;
  packet_rate = 11111.0;
  packet_end_time = 0.0;
  current_packet_time = 0.0;
  last_packet_time = 0.0;
  use_time_service = false;
  return_mode = 1;
  g_fAngleAcc_V = 0.01;
  packet_loss = false;
  is_add_frame_ = false;
  get_ms06_param = true;
  is_get_difop_ = false;
  last_packet_number_ = -1;
  current_packet_number_ = 0;
  total_packet_loss_ = 0;
  frame_count = 0;
  publish_thread_pool_ = std::make_unique<ThreadPool>(2);
  point_cloud_xyzirt_ = pcl::make_shared<pcl::PointCloud<PointXYZIRT>>();
  point_cloud_xyzirt_bak_ = pcl::make_shared<pcl::PointCloud<PointXYZIRT>>();
  point_cloud_xyzirt_pub_ = pcl::make_shared<pcl::PointCloud<PointXYZIRT>>();

  // create the sin and cos table for different azimuth and vertical values
  for (int j = 0; j < 36000; ++j) {
    double angle = static_cast<double>(j) * 0.01 * 0.017453293;
    sin_table[j] = sin(angle);
    cos_table[j] = cos(angle);
  }

  // mirror angle. the offset angle is different depending on the channel
  double mirror_angle[4] = {1.5, -0.5, 0.5, -1.5}; // double mirror_angle[4] = {0, -2, -1, -3};
  for (int i = 0; i < 4; ++i) {
    cos_mirror_angle[i] = cos(DEG2RAD(mirror_angle[i]));
    sin_mirror_angle[i] = sin(DEG2RAD(mirror_angle[i]));
  }
}

LsLidarDriver::~LsLidarDriver() {
  if (difop_thread_ && difop_thread_->joinable()) {
    AINFO << "Joining difop thread...";
    difop_thread_->join();
    AINFO << "difop thread joined.";
  }

  if (polling_thread_ && polling_thread_->joinable()) {
    AINFO << "Joining polling thread...";
    polling_thread_->join();
    AINFO << "polling thread joined.";
  }

  (void)close(socket_id);
}

bool LsLidarDriver::loadParameters() {
  packet_rate = config_.has_packet_rate() ? config_.packet_rate() : 11111.0;                                      // TODO: unused ?
  lidar_ip_string = config_.has_lidar_ip() ? config_.lidar_ip() : std::string("192.168.1.200");
  msop_udp_port = config_.has_msop_port() ? config_.msop_port() : (int)InputSocket::MSOP_DATA_PORT_NUMBER; 
  difop_udp_port = config_.has_difop_port() ? config_.difop_port() : (int)InputSocket::DIFOP_DATA_PORT_NUMBER;
  add_multicast = config_.has_add_multicast() ? config_.add_multicast() : false;
  group_ip_string = config_.has_group_ip() ? config_.group_ip() : std::string("224.1.1.2");
  min_range = config_.has_min_range() ? config_.min_range() : 0.5;
  max_range = config_.has_max_range() ? config_.max_range() : 1500.0;
  scan_start_angle = config_.has_scan_start_angle() ? config_.scan_start_angle() : -60;
  scan_end_angle = config_.has_scan_end_angle() ? config_.scan_end_angle() : 60;
  frame_id = config_.has_frame_id() ? config_.frame_id() : std::string("laser_link");
  pointcloud_channel = config_.has_pointcloud_channel() ? config_.pointcloud_channel() : std::string("lslidar_point_cloud"); // TODO: check it
  use_time_service = config_.has_use_time_service() ? config_.use_time_service() : false;
  packet_loss = config_.has_packet_loss() ? config_.packet_loss() : false;

  AINFO << "Using time service or not: " << use_time_service;
  AINFO << "Is packet loss detection enabled: " << packet_loss;
  AINFO << "Only accepting packets from IP address: " << lidar_ip_string.c_str();

  if (add_multicast) {
    AINFO << "Opening UDP socket: group_address " << group_ip_string;
  }

  return true;
}

bool LsLidarDriver::createCyberIO() {
  point_cloud_writer_ = node_->CreateWriter<apollo::drivers::PointCloud>(pointcloud_channel);

  if (packet_loss) {
    packet_loss_writer_ = node_->CreateWriter<Ls180s2PacketLoss>("packet_loss");
  }

  frame_rate_service_ = node_->CreateService<Ls180s2SrvFrameRate, Ls180s2SrvResult>("set_frame_rate",
                        std::bind(&LsLidarDriver::frameRate, this, std::placeholders::_1, std::placeholders::_2));
  data_ip_service_ = node_->CreateService<Ls180s2SrvDataIp, Ls180s2SrvResult>("set_data_ip",
                     std::bind(&LsLidarDriver::setDataIp, this, std::placeholders::_1, std::placeholders::_2));
  destination_ip_service_ = node_->CreateService<Ls180s2SrvDestinationIp, Ls180s2SrvResult>("set_destination_ip",
                            std::bind(&LsLidarDriver::setDestinationIp, this, std::placeholders::_1, std::placeholders::_2));
  data_port_service_ = node_->CreateService<Ls180s2SrvDataPort, Ls180s2SrvResult>("set_data_port",
                       std::bind(&LsLidarDriver::setDataPort, this, std::placeholders::_1, std::placeholders::_2));
  dev_port_service_ = node_->CreateService<Ls180s2SrvDevPort, Ls180s2SrvResult>("set_dev_port",
                      std::bind(&LsLidarDriver::setDevPort, this, std::placeholders::_1, std::placeholders::_2));

  msop_input_.reset(new ls180s2::InputSocket);
  if (!msop_input_->init(msop_udp_port, lidar_ip_string, group_ip_string, add_multicast)) {
    AERROR << "Failed to init msop input";
    return false;
  }

  difop_input_.reset(new ls180s2::InputSocket);
  if (!difop_input_->init(difop_udp_port, lidar_ip_string, group_ip_string, add_multicast)) {
    AERROR << "Failed to init difop input";
    return false;
  }

  difop_thread_ = std::make_unique<std::thread>(&LsLidarDriver::difopPoll, this);
  polling_thread_ = std::make_unique<std::thread>(&LsLidarDriver::dataPoll, this);

  return true;
}

void LsLidarDriver::initTimeStamp() {
    for (unsigned char &i : this->packetTimeStamp) {
        i = 0;
    }
    this->pointcloudTimeStamp = 0;
    this->timeStamp = apollo::cyber::Time(0.0);
}

bool LsLidarDriver::Init() {
  this->initTimeStamp();
  if (!loadParameters()) {
    AERROR << "Failed to init lslidar driver: can't load all required parameters.";
    return false;
  }

  if (!createCyberIO()) {
    AERROR << "Failed to init lslidar driver: can't create cyber IO.";
    return false;
  }

  return true;
}

void LsLidarDriver::dataPoll() {
  std::shared_ptr<Ls180s2Packet> packet(new Ls180s2Packet);

  while (!apollo::cyber::IsShutdown()) {
    // get lidar raw packet
    int rc = msop_input_->getPacket(packet.get());
    if (rc == 0) {
      // get packet data as uint8
      uint8_t* pck_data = (uint8_t*)packet->data().c_str();

      // publish message using time of last packet read
      if (use_time_service) {
        if (0xff == pck_data[1194]) {
          uint64_t timestamp_s = (pck_data[1195] * 0 + (pck_data[1196] << 24) +
                                  (pck_data[1197] << 16) + (pck_data[1198] << 8) + pck_data[1199] * pow(2, 0));
          uint64_t timestamp_nsce = (pck_data[1200] << 24) + (pck_data[1201] << 16) +
                                    (pck_data[1202] << 8) + (pck_data[1203]);
          timeStamp = apollo::cyber::Time(timestamp_s, timestamp_nsce);
          packet->set_stamp(timeStamp.ToNanosecond());
          current_packet_time = timeStamp.ToSecond();
        }
        else {
          this->packetTimeStamp[4] = pck_data[1199];
          this->packetTimeStamp[5] = pck_data[1198];
          this->packetTimeStamp[6] = pck_data[1197];
          this->packetTimeStamp[7] = pck_data[1196];
          this->packetTimeStamp[8] = pck_data[1195];
          this->packetTimeStamp[9] = pck_data[1194];

          struct tm cur_time{};
          memset(&cur_time, 0, sizeof(cur_time));
          cur_time.tm_sec = this->packetTimeStamp[4];
          cur_time.tm_min = this->packetTimeStamp[5];
          cur_time.tm_hour = this->packetTimeStamp[6];
          cur_time.tm_mday = this->packetTimeStamp[7];
          cur_time.tm_mon = this->packetTimeStamp[8] - 1;
          cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;
          this->pointcloudTimeStamp = static_cast<uint64_t>(timegm(&cur_time)); // seconds

          uint64_t packet_timestamp;
          packet_timestamp = pck_data[1203] +
                              (pck_data[1202] << 8) +
                              (pck_data[1201] << 16) +
                              (pck_data[1200] << 24); // nanoseconds
          timeStamp = apollo::cyber::Time(this->pointcloudTimeStamp, packet_timestamp);
          packet->set_stamp(timeStamp.ToNanosecond());
          current_packet_time = timeStamp.ToSecond();
        }
      }
      else {
        auto now_ts = apollo::cyber::Time::Now();
        packet->set_stamp(now_ts.ToNanosecond());
        current_packet_time = now_ts.ToSecond();
      }

      lslidarChPacketProcess(packet);
    }
    if (rc < 0) {
      break;
    }
  }

  AINFO << "dataPoll finished";
}

void LsLidarDriver::lslidarChPacketProcess(const std::shared_ptr<Ls180s2Packet> &packet) {
  struct Firing lidardata{};

  // convert the msg to the raw packet type
  packet_end_time = apollo::cyber::Time(packet->stamp()).ToSecond();
  uint8_t* pck_data = (uint8_t*)packet->data().c_str();

  bool packetType = false;
  if (pck_data[1205] == 0x02) {
    return_mode = 2;
  }

  if(get_ms06_param && m_horizontal_point != 0 && pck_data[1204] == 192) {
    double mirror_angle[4] = {1.5, 0.5, -0.5, -1.5}; // TODO: as in constructor. move to common field

    for (int i = 0; i < 4; ++i) {
      cos_mirror_angle[i] = cos(DEG2RAD(mirror_angle[i]));
      sin_mirror_angle[i] = sin(DEG2RAD(mirror_angle[i]));
    }

    m_offset = 10.82;
    g_fAngleAcc_V = 0.01;
    g_fDistanceAcc = 0.1 * 0.04;
    get_ms06_param = false;
  }

  if (return_mode == 1) {
    if (packet_loss) {
      current_packet_number_ = (pck_data[1192] << 8) + pck_data[1193];
      tmp_packet_number_ = current_packet_number_;

      if(current_packet_number_ - last_packet_number_ < 0) {
        current_packet_number_ += 65536;
      }

      if (current_packet_number_ - last_packet_number_ > 1 && last_packet_number_ != -1) {
        total_packet_loss_ += current_packet_number_ - last_packet_number_ - 1;
        ADEBUG << "packet loss = " << total_packet_loss_;

        Ls180s2PacketLoss loss_data;
        loss_data.set_count(total_packet_loss_);
        packet_loss_writer_->Write(loss_data);
      }
      last_packet_number_ = tmp_packet_number_;
    }

    double packet_interval_time =
            (current_packet_time - last_packet_time) / (POINTS_BYTES_PER_PACKET_SINGLE_ECHO / 8.0);

    for (size_t point_idx = 0; point_idx < POINTS_BYTES_PER_PACKET_SINGLE_ECHO; point_idx += 8) {
      if ((pck_data[point_idx] == 0xff) && (pck_data[point_idx + 1] == 0xaa) &&
          (pck_data[point_idx + 2] == 0xbb) && (pck_data[point_idx + 3] == 0xcc) &&
          (pck_data[point_idx + 4] == 0xdd)) {

          packetType = true;
          frame_count++;
      }
      else {
        // compute the time of the point
        double point_time;
        if (last_packet_time > 1e-6) {
          point_time = packet_end_time -
                       packet_interval_time * ((POINTS_BYTES_PER_PACKET_SINGLE_ECHO - point_idx) / 8 - 1);
        }
        else {
          point_time = current_packet_time;
        }

        memset(&lidardata, 0, sizeof(lidardata));

        // horizontal angle
        double fAngle_H = pck_data[point_idx + 1] + (pck_data[point_idx] << 8);
        if (fAngle_H > 32767) {
          fAngle_H = (fAngle_H - 65536);
        }

        lidardata.azimuth = fAngle_H * 0.01;

        // vertical angle + channel number
        int iTempAngle = pck_data[point_idx + 2];
        int iChannelNumber = iTempAngle >> 6; // shift left six bits channel number
        int iSymmbol = (iTempAngle >> 5) & 0x01; // shift left five bits sign bit
        double fAngle_V = 0.0;
        if (1 == iSymmbol) { // sign bit (0 - positive, 1 - negative)
          int iAngle_V = pck_data[point_idx + 3] + (pck_data[point_idx + 2] << 8);
          fAngle_V = iAngle_V | 0xc000;
          if (fAngle_V > 32767) {
            fAngle_V = (fAngle_V - 65536);
          }
        }
        else {
          int iAngle_Hight = iTempAngle & 0x3f;
          fAngle_V = pck_data[point_idx + 3] + (iAngle_Hight << 8);
        }

        lidardata.vertical_angle = fAngle_V * g_fAngleAcc_V;
        if ((lidardata.azimuth < scan_start_angle) || (lidardata.azimuth > scan_end_angle)) {
          continue;
        }

        lidardata.channel_number = iChannelNumber;
        lidardata.distance = ((pck_data[point_idx + 4] << 16) + (pck_data[point_idx + 5] << 8) + pck_data[point_idx + 6]);
        if (!isPointInRange(lidardata.distance * g_fDistanceAcc)) {
          continue;
        }

        lidardata.intensity = pck_data[point_idx + 7];
        lidardata.time = point_time;
        lidardata.azimuth = fAngle_H * 0.01;
        convertCoordinate(lidardata);
      }

      if (packetType) {
        if (is_add_frame_) {
          if (frame_count >= 2) {
            {
              std::unique_lock<std::mutex> lock(pc_mutex_);
              point_cloud_xyzirt_pub_ = point_cloud_xyzirt_;
            }
            publish_thread_pool_->enqueue([&]() { publishPointCloudNew(); });
          }
          packetType = false;
          point_cloud_xyzirt_ = point_cloud_xyzirt_bak_;
          point_cloud_xyzirt_bak_.reset(new pcl::PointCloud<PointXYZIRT>);
        }
        else {
          {
            std::unique_lock<std::mutex> lock(pc_mutex_);
            point_cloud_xyzirt_pub_ = point_cloud_xyzirt_;
          }
          publish_thread_pool_->enqueue([&]() { publishPointCloudNew(); });
          packetType = false;
          point_cloud_xyzirt_.reset(new pcl::PointCloud<PointXYZIRT>);
          point_cloud_xyzirt_bak_.reset(new pcl::PointCloud<PointXYZIRT>);
        }
      }
    }
  }
  else {
    if (packet_loss) {
      current_packet_number_ = (pck_data[1188] * 1099511627776) + (pck_data[1189] * 4294967296) +
                                (pck_data[1190] * 16777216) + (pck_data[1191] * 65536) +
                                (pck_data[1192] * 256) + pck_data[1193];

      if (current_packet_number_ - last_packet_number_ > 1 && last_packet_number_ != -1) {
          total_packet_loss_ += current_packet_number_ - last_packet_number_ - 1;
          ADEBUG << "packet loss = " << total_packet_loss_;
          Ls180s2PacketLoss loss_data;
          loss_data.set_count(total_packet_loss_);
          packet_loss_writer_->Write(loss_data);
      }
      last_packet_number_ = current_packet_number_;
    }

    double packet_interval_time =
            (current_packet_time - last_packet_time) / (POINTS_BYTES_PER_PACKET_DOUBLE_ECHO / 12.0);

    for (size_t point_idx = 0; point_idx < POINTS_BYTES_PER_PACKET_DOUBLE_ECHO; point_idx += 12) {
      if ((pck_data[point_idx] == 0xff) && (pck_data[point_idx + 1] == 0xaa) &&
          (pck_data[point_idx + 2] == 0xbb) && (pck_data[point_idx + 3] == 0xcc) &&
          (pck_data[point_idx + 4] == 0xdd)) {

        packetType = true;
        frame_count++;
      }
      else {
        // compute the time of the point
        double point_time;
        if (last_packet_time > 1e-6) {
          point_time = packet_end_time -
                        packet_interval_time * ((POINTS_BYTES_PER_PACKET_DOUBLE_ECHO - point_idx) / 12 - 1);
        }
        else {
          point_time = current_packet_time;
        }

        memset(&lidardata, 0, sizeof(lidardata));

        // horizontal angle
        double fAngle_H = pck_data[point_idx + 1] + (pck_data[point_idx] << 8);
        if (fAngle_H > 32767) {
            fAngle_H = (fAngle_H - 65536);
        }
        lidardata.azimuth = fAngle_H * 0.01;

        // vertical angle + channel number
        int iTempAngle = pck_data[point_idx + 2];
        int iChannelNumber = iTempAngle >> 6; // shift left six bits - channel number
        int iSymmbol = (iTempAngle >> 5) & 0x01; // Shift left five bits - sign bit
        double fAngle_V = 0.0;

        if (1 == iSymmbol) { // sign bit (0 - positive, 1 - negative)
          int iAngle_V = pck_data[point_idx + 3] + (pck_data[point_idx + 2] << 8);
          fAngle_V = iAngle_V | 0xc000;
          if (fAngle_V > 32767) {
              fAngle_V = (fAngle_V - 65536);
          }
        }
        else {
          int iAngle_Hight = iTempAngle & 0x3f;
          fAngle_V = pck_data[point_idx + 3] + (iAngle_Hight << 8);
        }

        lidardata.vertical_angle = fAngle_V * g_fAngleAcc_V;
        if ((lidardata.azimuth < scan_start_angle) || (lidardata.azimuth > scan_end_angle)) {
          continue;
        }

        lidardata.channel_number = iChannelNumber;
        lidardata.distance = ((pck_data[point_idx + 4] << 16) + (pck_data[point_idx + 5] << 8) + pck_data[point_idx + 6]);
        if (!isPointInRange(lidardata.distance * g_fDistanceAcc)) {
          continue;
        }

        lidardata.intensity = pck_data[point_idx + 7];
        lidardata.time = point_time;
        convertCoordinate(lidardata); // first point

        lidardata.distance = ((pck_data[point_idx + 8] << 16) + (pck_data[point_idx + 9] << 8) + pck_data[point_idx + 10]);
        if (!isPointInRange(lidardata.distance * g_fDistanceAcc)) {
          continue;
        }

        lidardata.intensity = pck_data[point_idx + 11];
        lidardata.time = point_time;
        convertCoordinate(lidardata); // second point
      }

      if (packetType) {
        if (is_add_frame_) {
            if (frame_count >= 2) {
              {
                std::unique_lock<std::mutex> lock(pc_mutex_);
                point_cloud_xyzirt_pub_ = point_cloud_xyzirt_;
              }
              publish_thread_pool_->enqueue([&]() { publishPointCloudNew(); });
            }
            packetType = false;
            point_cloud_xyzirt_ = point_cloud_xyzirt_bak_;
            point_cloud_xyzirt_bak_.reset(new pcl::PointCloud<PointXYZIRT>);
        }
        else {
          {
              std::unique_lock<std::mutex> lock(pc_mutex_);
              point_cloud_xyzirt_pub_ = point_cloud_xyzirt_;
          }
          publish_thread_pool_->enqueue([&]() { publishPointCloudNew(); });
          packetType = false;
          point_cloud_xyzirt_.reset(new pcl::PointCloud<PointXYZIRT>);
          point_cloud_xyzirt_bak_.reset(new pcl::PointCloud<PointXYZIRT>);
        }
      }
    }
  }

  last_packet_time = current_packet_time;
}

int LsLidarDriver::convertCoordinate(const struct Firing &lidardata) {
  double fAngle_H = 0.0; // horizontal angle
  double fAngle_V = 0.0; // vertical angle
  fAngle_H = lidardata.azimuth;
  fAngle_V = lidardata.vertical_angle;

  // add distortion
  double fSinV_angle = 0;
  double fCosV_angle = 0;

  // galvanometer offset angle = actual vertical angle / 2  - offset value
  double fGalvanometrtAngle = 0;
  //fGalvanometrtAngle = (((fAngle_V + 0.05) / 0.8) + 1) * 0.46 + 6.72;
  //fGalvanometrtAngle = fAngle_V + 7.26;
  //fGalvanometrtAngle = fAngle_V + 6.37;
  fGalvanometrtAngle = fAngle_V + m_offset;

  while (fGalvanometrtAngle < 0.0) {
      fGalvanometrtAngle += 360.0;
  }

  while (fAngle_H < 0.0) {
      fAngle_H += 360.0;
  }

  int table_index_V = int(fGalvanometrtAngle * 100) % 36000;
  int table_index_H = int(fAngle_H * 100) % 36000;

  // double fAngle_R0 = cos30 * cos_mirror_angle[lidardata.channel_number % 4] * cos_table[table_index_V] -
  //                     sin_table[table_index_V] * sin_mirror_angle[lidardata.channel_number % 4];
  double fAngle_R0 = cos(DEG2RAD(30)) * cos_mirror_angle[lidardata.channel_number % 4] * cos_table[table_index_V] -
                      sin_table[table_index_V] * sin_mirror_angle[lidardata.channel_number % 4];

  fSinV_angle = 2 * fAngle_R0 * sin_table[table_index_V] + sin_mirror_angle[lidardata.channel_number % 4];
  fCosV_angle = sqrt(1 - pow(fSinV_angle, 2));

  // double fSinCite = (2 * fAngle_R0 * cos_table[table_index_V] * sin30 -
  //                     cos_mirror_angle[lidardata.channel_number % 4] * sin60) / fCosV_angle;
  double fSinCite = (2 * fAngle_R0 * cos_table[table_index_V] * sin(DEG2RAD(30)) -
                      cos_mirror_angle[lidardata.channel_number % 4] * sin(DEG2RAD(60))) / fCosV_angle;

  double fCosCite = sqrt(1 - pow(fSinCite, 2));

  double fSinCite_H = sin_table[table_index_H] * fCosCite + cos_table[table_index_H] * fSinCite;
  double fCosCite_H = cos_table[table_index_H] * fCosCite - sin_table[table_index_H] * fSinCite;

  double x_coord = 0.0, y_coord = 0.0, z_coord = 0.0;
  x_coord = (lidardata.distance * fCosV_angle * fSinCite_H) * g_fDistanceAcc;
  y_coord = (lidardata.distance * fCosV_angle * fCosCite_H) * g_fDistanceAcc;
  z_coord = (lidardata.distance * fSinV_angle) * g_fDistanceAcc;

  PointXYZIRT point;
  point.x = x_coord;
  point.y = y_coord;
  point.z = z_coord;
  point.intensity = lidardata.intensity;
  point.ring = lidardata.channel_number;
  point.timestamp = lidardata.time;

  point_cloud_xyzirt_->points.push_back(point);
  ++point_cloud_xyzirt_->width;

  point_cloud_xyzirt_bak_->points.push_back(point);
  ++point_cloud_xyzirt_bak_->width;

  return 0;
}

void LsLidarDriver::publishPointCloudNew() {
  if (!is_get_difop_) {
    return;
  }

  AINFO << "Ready to publish: " << point_cloud_xyzirt_pub_->size();

  // copy PCL points
  std::unique_lock<std::mutex> lock(pc_mutex_);
  const auto points_copy = point_cloud_xyzirt_pub_->points;
  const auto width = point_cloud_xyzirt_pub_->width;
  lock.unlock();

  // // remove points
  // const auto points_overhead = (int)points_copy.size() - MAX_POINTS_TO_PUBLISH;
  // if (points_overhead > 0) {
  //   AWARN << "Remove overhead points: " << points_copy.size() << "/" << MAX_POINTS_TO_PUBLISH;
  //   points_copy.erase(points_copy.begin(), points_copy.begin() + points_overhead);
  // }
  // assert(points_copy.size() <= MAX_POINTS_TO_PUBLISH);

  // prepare message to publish
  apollo::drivers::PointCloud result;
  result.mutable_point()->Reserve(points_copy.size());

  apollo::cyber::Time last_timestamp(0);

  for (auto& point : point_cloud_xyzirt_pub_->points) {
    auto* res_point = result.add_point();
    res_point->set_x(point.x);
    res_point->set_y(point.y);
    res_point->set_z(point.z);
    res_point->set_intensity((uint32_t)point.intensity);

    const auto cur_timestamp = cyber::Time(point.timestamp);
    if (cur_timestamp > last_timestamp) {
      last_timestamp = cur_timestamp;
    }
    res_point->set_timestamp(cur_timestamp.ToNanosecond());
  }

  if (last_timestamp.IsZero()) {
    AERROR << "Failed to set valid timestamp to packet";
  }

  result.set_width(width);
  result.set_height(1);
  result.set_frame_id(frame_id);
  result.set_is_dense(false);
  result.set_measurement_time(last_timestamp.ToSecond());

  result.mutable_header()->set_frame_id(frame_id);
  result.mutable_header()->set_module_name(node_->Name());
  result.mutable_header()->set_sequence_num(sequence_num.fetch_add(1));
  result.mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond()); // publish time
  result.mutable_header()->set_lidar_timestamp(last_timestamp.ToNanosecond());

  // publish message
  point_cloud_writer_->Write(result);
}

void LsLidarDriver::difopPoll() {
  // reading and publishing scans as fast as possible.
  std::shared_ptr<Ls180s2Packet> difop_packet(new Ls180s2Packet);

  while(!apollo::cyber::IsShutdown()) {
    // keep reading
    int rc = difop_input_->getPacket(difop_packet.get());
    if (rc == 0) {
      uint8_t* pck_data = (uint8_t*)difop_packet->data().c_str();

      if (pck_data[0] == 0x00 || pck_data[0] == 0xa5) {
        if (pck_data[1] == 0xff && pck_data[2] == 0x00 && pck_data[3] == 0x5a) {
          if (pck_data[231] == 64 || pck_data[231] == 65) {
            is_add_frame_ = true;
          }

          for (int i = 0; i < 1206; i++) {
              difop_data[i] = pck_data[i];
          }

          m_horizontal_point = pck_data[184] * 256 + pck_data[185];
          int majorVersion = pck_data[1202];
          int minorVersion1 = pck_data[1203] / 16;
          // int minorVersion2 = pck_data[1203] % 16; // TODO: unused

          //v1.1 :0.01   //v1.2以后  ： 0.0025
          if (1 > majorVersion || (1 == majorVersion && minorVersion1 > 1)) {
            g_fAngleAcc_V = 0.0025;
          }
          else {
            g_fAngleAcc_V = 0.01;
          }

          float fInitAngle_V = pck_data[188] * 256 + pck_data[189];
          if (fInitAngle_V > 32767) {
              fInitAngle_V = fInitAngle_V - 65536;
          }
          this->prism_angle[0] = fInitAngle_V * g_fAngleAcc_V;

          fInitAngle_V = pck_data[190] * 256 + pck_data[191];
          if (fInitAngle_V > 32767) {
              fInitAngle_V = fInitAngle_V - 65536;
          }
          this->prism_angle[1] = fInitAngle_V * g_fAngleAcc_V;

          fInitAngle_V = pck_data[192] * 256 + pck_data[193];
          if (fInitAngle_V > 32767) {
              fInitAngle_V = fInitAngle_V - 65536;
          }
          this->prism_angle[2] = fInitAngle_V * g_fAngleAcc_V;

          fInitAngle_V = pck_data[194] * 256 + pck_data[195];
          if (fInitAngle_V > 32767) {
              fInitAngle_V = fInitAngle_V - 65536;
          }
          this->prism_angle[3] = fInitAngle_V * g_fAngleAcc_V;
          is_get_difop_ = true;
        }
      }
    }
    else if (rc < 0) {
      break;
    }
  }

  AINFO << "difopPoll finished";
}

bool LsLidarDriver::frameRate(const std::shared_ptr<Ls180s2SrvFrameRate>& req, std::shared_ptr<Ls180s2SrvResult>& res) {
  if (!is_get_difop_) {
      res->set_result(false);
      std::cout << "\033[1m\033[33m" << "Can not get dev packet! Set failed!"<< "\033[0m" << std::endl;
      return true;
  }

  unsigned char config_data[1206];
  mempcpy(config_data, difop_data, 1206);
  if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
      res->set_result(false);
      std::cout << "\033[1m\033[33m" << "Can not get dev packet! Set failed!"<< "\033[0m" << std::endl;
      return true;
  }

  setPacketHeader(config_data);
  std::string frame_rate_ = "";
  if (req->frame_rate() == 0) {
      config_data[100] = 0x00;
      frame_rate_ = "Standard frame rate";
  }
  else if (req->frame_rate() == 1) {
      config_data[100] = 0x01;
      frame_rate_ = "50 percent frame rate";
  }
  else if (req->frame_rate() == 2) {
      config_data[100] = 0x02;
      frame_rate_ = "25 percent frame rate";
  }
  else {
      std::cout << "\033[1m\033[31m" << "Parameter error, please check the input parameters" << "\033[0m" << std::endl;
      res->set_result(false);
      return true;
  }
  res->set_result(true);
  sendPacketTolidar(config_data);

  std::cout << "\033[1m\033[32m" << "------------------------------------------------------------" << "\033[0m" << std::endl;
  std::cout << "\033[1m\033[32m" <<"Set successfully! Current lidar " << frame_rate_ << "\033[0m" << std::endl;
  
  return true;
}

bool LsLidarDriver::setDataIp(const std::shared_ptr<Ls180s2SrvDataIp>& req, std::shared_ptr<Ls180s2SrvResult>& res) {
  std::regex ipv4("\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");

  if (!regex_match(req->data_ip(), ipv4)) {
    std::cout << "\033[1m\033[31m" << "Parameter error, please check the input parameters" << "\033[0m" << std::endl;
    res->set_result(false);
    return true;
  }

  if (!is_get_difop_) {
    res->set_result(false);
    std::cout << "\033[1m\033[33m" << "Can not get dev packet! Set failed!"<< "\033[0m" << std::endl;
    return true;
  }

  unsigned char config_data[1206];
  mempcpy(config_data, difop_data, 1206);
  if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
    res->set_result(false);
    std::cout << "\033[1m\033[33m" << "Can not get dev packet! Set failed!"<< "\033[0m" << std::endl;
    return true;
  }
  setPacketHeader(config_data);
  is_get_difop_ = false;

  unsigned short first_value, second_value, third_value, end_value;
  sscanf(req->data_ip().c_str(), "%hu.%hu.%hu.%hu", &first_value, &second_value, &third_value, &end_value);

  std::string destination_ip = std::to_string(config_data[14]) + "." + std::to_string(config_data[15]) + "." +
                                std::to_string(config_data[16]) + "." + std::to_string(config_data[17]);

  if (first_value == 0 || first_value == 127 || (first_value >= 240 && first_value <= 255) || destination_ip == req->data_ip()) {
    std::cout << "\033[1m\033[31m" << "Parameter error, please check the input parameters" << "\033[0m" << std::endl;
    res->set_result(false);
    return true;
  }
  else {
    config_data[10] = first_value;
    config_data[11] = second_value;
    config_data[12] = third_value;
    config_data[13] = end_value;
  }
  res->set_result(true);
  sendPacketTolidar(config_data);

  std::cout << "\033[1m\033[32m" << "------------------------------------------------------------" << "\033[0m" << std::endl;
  std::cout << "\033[1m\033[32m" <<"Set successfully! Current lidar ip:" << req->data_ip().c_str() << "\033[0m" << std::endl;
  std::cout << "\033[1m\033[32m" <<"Please modify the corresponding parameters in the launch file" << "\033[0m" << std::endl;

  return true;
}

bool LsLidarDriver::setDestinationIp(const std::shared_ptr<Ls180s2SrvDestinationIp>& req, std::shared_ptr<Ls180s2SrvResult>& res) {
  std::regex ipv4("\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");

  if (!regex_match(req->dest_ip(), ipv4)) {
    std::cout << "\033[1m\033[31m" << "Parameter error, please check the input parameters" << "\033[0m" << std::endl;
    res->set_result(false);
    return true;
  }

  if (!is_get_difop_) {
    res->set_result(false);
    std::cout << "\033[1m\033[33m" << "Can not get dev packet! Set failed!"<< "\033[0m" << std::endl;
    return true;
  }

  unsigned char config_data[1206];
  mempcpy(config_data, difop_data, 1206);
  if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
    res->set_result(false);
    std::cout << "\033[1m\033[33m" << "Can not get dev packet! Set failed!"<< "\033[0m" << std::endl;
    return true;
  }

  setPacketHeader(config_data);
  is_get_difop_ = false;
  unsigned short first_value, second_value, third_value, end_value;
  sscanf(req->dest_ip().c_str(), "%hu.%hu.%hu.%hu", &first_value, &second_value, &third_value, &end_value);

  std::string data_ip = std::to_string(config_data[10]) + "." + std::to_string(config_data[11]) + "." +
                        std::to_string(config_data[12]) + "." + std::to_string(config_data[13]);

  if (first_value == 0 || first_value == 127 || (first_value >= 240 && first_value <= 255) || data_ip == req->dest_ip()) {
      std::cout << "\033[1m\033[31m" << "Parameter error, please check the input parameters" << "\033[0m" << std::endl;
      res->set_result(false);
      return true;
  }
  else {
      config_data[14] = first_value;
      config_data[15] = second_value;
      config_data[16] = third_value;
      config_data[17] = end_value;
  }

  res->set_result(true);
  sendPacketTolidar(config_data);

  std::cout << "\033[1m\033[32m" << "------------------------------------------------------------" << "\033[0m" << std::endl;
  std::cout << "\033[1m\033[32m" <<"Set successfully! Current lidar destination ip:" << req->dest_ip().c_str() << "\033[0m" << std::endl;
  std::cout << "\033[1m\033[32m" <<"Please modify the local IP address" << "\033[0m" << std::endl;

  return true;
}

bool LsLidarDriver::setDataPort(const std::shared_ptr<Ls180s2SrvDataPort>& req, std::shared_ptr<Ls180s2SrvResult>& res) {
  if (!is_get_difop_) {
    res->set_result(false);
    std::cout << "\033[1m\033[33m" << "Can not get dev packet! Set failed!"<< "\033[0m" << std::endl;
    return true;
  }

  unsigned char config_data[1206];
  mempcpy(config_data, difop_data, 1206);
  if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
    res->set_result(false);
    std::cout << "\033[1m\033[33m" << "Can not get dev packet! Set failed!"<< "\033[0m" << std::endl;
    return true;
  }

  setPacketHeader(config_data);
  is_get_difop_ = false;
  int dev_port = config_data[26] * 256 + config_data[27];
  if (req->data_port() < 1025 || req->data_port() > 65535 || req->data_port() == dev_port) {
    std::cout << "\033[1m\033[31m" << "Parameter error, please check the input parameters" << "\033[0m" << std::endl;
    res->set_result(false);
    return true;
  }
  else {
    config_data[24] = req->data_port() / 256;
    config_data[25] = req->data_port() % 256;
  }
  res->set_result(true);
  sendPacketTolidar(config_data);

  std::cout << "\033[1m\033[32m" << "------------------------------------------------------------" << "\033[0m" << std::endl;
  std::cout << "\033[1m\033[32m" <<"Set successfully! Current lidar MSOP port:" << req->data_port() << "\033[0m" << std::endl;
  std::cout << "\033[1m\033[32m" <<"Please modify the corresponding parameters in the launch file" << "\033[0m" << std::endl;

  return true;
}

bool LsLidarDriver::setDevPort(const std::shared_ptr<Ls180s2SrvDevPort>& req, std::shared_ptr<Ls180s2SrvResult>& res) {
  if (!is_get_difop_) {
    res->set_result(false);
    std::cout << "\033[1m\033[33m" << "Can not get dev packet! Set failed!"<< "\033[0m" << std::endl;
    return true;
  }

  unsigned char config_data[1206];
  mempcpy(config_data, difop_data, 1206);
  if (config_data[0] != 0xa5 || config_data[1] != 0xff || config_data[2] != 0x00 || config_data[3] != 0x5a) {
    res->set_result(false);
    std::cout << "\033[1m\033[33m" << "Can not get dev packet! Set failed!"<< "\033[0m" << std::endl;
    return true;
  }
  setPacketHeader(config_data);
  is_get_difop_ = false;

  int data_port = config_data[24] * 256 + config_data[25];
  if (req->dev_port() < 1025 || req->dev_port() > 65535 || req->dev_port() == data_port) {
    std::cout << "\033[1m\033[31m" << "Parameter error, please check the input parameters" << "\033[0m" << std::endl;
    res->set_result(false);
    return true;
  }
  else {
    config_data[26] = req->dev_port() / 256;
    config_data[27] = req->dev_port() % 256;
  }
  res->set_result(true);
  sendPacketTolidar(config_data);

  std::cout << "\033[1m\033[32m" << "------------------------------------------------------------" << "\033[0m" << std::endl;
  std::cout << "\033[1m\033[32m" <<"Set successfully! Current lidar DIFOP port:" << req->dev_port() << "\033[0m" << std::endl;
  std::cout << "\033[1m\033[32m" <<"Please modify the corresponding parameters in the launch file" << "\033[0m" << std::endl;

  return true;
}

void LsLidarDriver::setPacketHeader(unsigned char *config_data) {
  config_data[0] = 0xAA;
  config_data[1] = 0x00;
  config_data[2] = 0xFF;
  config_data[3] = 0x11;
  config_data[4] = 0x22;
  config_data[5] = 0x22;
  config_data[6] = 0xAA;
  config_data[7] = 0xAA;
}

bool LsLidarDriver::sendPacketTolidar(unsigned char *config_data) const { 
  int socketid;
  sockaddr_in addrSrv{};
  socketid = socket(2, 2, 0);
  addrSrv.sin_addr.s_addr = inet_addr(lidar_ip_string.c_str());
  addrSrv.sin_family = AF_INET;
  addrSrv.sin_port = htons(InputSocket::MSOP_DATA_PORT_NUMBER);
  sendto(socketid, (const char *) config_data, InputSocket::PACKET_SIZE, 0, (struct sockaddr *) &addrSrv, sizeof(addrSrv));
  return true;
}

}  // namespace ls180s2
}  // namespace drivers
}  // namespace apollo
