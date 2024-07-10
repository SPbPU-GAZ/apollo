/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/drivers/gnss/parser/data_parser.h"

#include <cmath>
#include <memory>
#include <string>

#include "Eigen/Geometry"
#include "boost/array.hpp"
#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"
#include "modules/common_msgs/sensor_msgs/gnss_best_pose.pb.h"
#include "modules/common_msgs/sensor_msgs/gnss_raw_observation.pb.h"
#include "modules/common_msgs/sensor_msgs/heading.pb.h"
#include "modules/common_msgs/localization_msgs/imu.pb.h"
#include "modules/common_msgs/sensor_msgs/gkv_nav.pb.h"

#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/util/time_conversion.h"
#include "modules/drivers/gnss/util/gnss_conversion.h"

namespace apollo {
namespace drivers {
namespace gnss {

using ::apollo::localization::CorrectedImu;
using ::apollo::localization::Gps;

using apollo::transform::TransformStamped;

namespace {

constexpr double DEG_TO_RAD_LOCAL = M_PI / 180.0;
const char *WGS84_TEXT = "+proj=latlong +ellps=WGS84";

// covariance data for pose if can not get from novatel inscov topic
static const boost::array<double, 36> POSE_COVAR = {
    2, 0, 0, 0,    0, 0, 0, 2, 0, 0, 0,    0, 0, 0, 2, 0, 0, 0,
    0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0.01};

Parser *CreateParser(config::Config config, bool is_base_station = false) {
  switch (config.data().format()) {
    case config::Stream::NOVATEL_BINARY:
      return Parser::CreateNovatel(config);
    case config::Stream::GKV_BINARY:
      return Parser::CreateGkv(config);
    default:
      return nullptr;
  }
}

// North/East/Down -> East/North/Up
inline void ned_to_enu(double n, double e, double d, ::apollo::common::Point3D* enu) {
  enu->set_x(e);
  enu->set_y(n);
  enu->set_z(-d);
}

static std::string uint8_to_hex_string(const uint8_t *v, const size_t s) {
  std::stringstream ss;
  ss << std::uppercase << std::hex << std::setfill('0');
  for (size_t i = 0; i < s; i++) {
    ss << std::uppercase << std::hex << std::setw(2) << static_cast<int>(v[i]);
  }

  return ss.str();
}

}  // namespace

DataParser::DataParser(const config::Config &config,
                       const std::shared_ptr<apollo::cyber::Node> &node)
    : config_(config), tf_broadcaster_(node), node_(node) {
  std::string utm_target_param;

  wgs84pj_source_ = pj_init_plus(WGS84_TEXT);
  utm_target_ = pj_init_plus(config_.proj4_text().c_str());
  gnss_status_.set_solution_status(0);
  gnss_status_.set_num_sats(0);
  gnss_status_.set_position_type(0);
  gnss_status_.set_solution_completed(false);
  ins_status_.set_type(InsStatus::INVALID);
}

bool DataParser::Init() {
  // // ------------- GPRMC TEST ---------------
  // const auto time_now_ns = cyber::Time::Now().ToNanosecond();
  // const double lat_deg = 60.767416;
  // const double lon_deg = 30.375744;
  // const auto gprmc_str = PrepareGPRMC(time_now_ns, lat_deg, lon_deg, true, false);
  // AWARN << "GPRMC: " << gprmc_str.c_str();
  // // ------------- ---------- ---------------

  ins_status_.mutable_header()->set_timestamp_sec(
      cyber::Time::Now().ToSecond());
  gnss_status_.mutable_header()->set_timestamp_sec(
      cyber::Time::Now().ToSecond());

  gnssstatus_writer_ = node_->CreateWriter<GnssStatus>(FLAGS_gnss_status_topic);
  insstatus_writer_ = node_->CreateWriter<InsStatus>(FLAGS_ins_status_topic);
  gnssbestpose_writer_ =
      node_->CreateWriter<GnssBestPose>(FLAGS_gnss_best_pose_topic);
  corrimu_writer_ = node_->CreateWriter<CorrectedImu>(FLAGS_imu_topic);
  insstat_writer_ = node_->CreateWriter<InsStat>(FLAGS_ins_stat_topic);
  gnssephemeris_writer_ =
      node_->CreateWriter<GnssEphemeris>(FLAGS_gnss_rtk_eph_topic);
  epochobservation_writer_ =
      node_->CreateWriter<EpochObservation>(FLAGS_gnss_rtk_obs_topic);
  heading_writer_ = node_->CreateWriter<Heading>(FLAGS_heading_topic);
  rawimu_writer_ = node_->CreateWriter<Imu>(FLAGS_raw_imu_topic);
  gps_writer_ = node_->CreateWriter<Gps>(FLAGS_gps_topic);

  common::util::FillHeader("gnss", &ins_status_);
  insstatus_writer_->Write(ins_status_);
  common::util::FillHeader("gnss", &gnss_status_);
  gnssstatus_writer_->Write(gnss_status_);

  AINFO << "Creating data parser of format: " << config_.data().format();
  data_parser_.reset(CreateParser(config_, false));
  if (!data_parser_) {
    AFATAL << "Failed to create data parser.";
    return false;
  }

  init_flag_ = true;
  return true;
}

void DataParser::ParseRawData(const std::string &msg) {
  if (!init_flag_) {
    AERROR << "Data parser not init.";
    return;
  }

  data_parser_->Update(msg);
  Parser::MessageType type;
  MessagePtr msg_ptr;

  while (cyber::OK()) {
    type = data_parser_->GetMessage(&msg_ptr);
    if (type == Parser::MessageType::NONE) {
      break;
    }
    DispatchMessage(type, msg_ptr);
  }
}

std::string DataParser::GetLastGPRMC() {
  std::string result;

  if (!init_flag_) {
    AERROR << "Data parser not init.";
    return result;
  }

  // {
  //   std::lock_guard<std::mutex> gprmc_lock(gprmc_str_mutex_);
  //   result = last_gprmc_str_;
  // }

  result = last_gprmc_str_;

  return result;
}

void DataParser::CheckInsStatus(::apollo::drivers::gnss::Ins *ins) {
  static double last_notify = cyber::Time().Now().ToSecond();
  double now = cyber::Time().Now().ToSecond();
  if (ins_status_record_ != static_cast<uint32_t>(ins->type()) ||
      (now - last_notify) > 1.0) {
    last_notify = now;
    ins_status_record_ = static_cast<uint32_t>(ins->type());
    switch (ins->type()) {
      case apollo::drivers::gnss::Ins::GOOD:
        ins_status_.set_type(apollo::drivers::gnss::InsStatus::GOOD);
        break;

      case apollo::drivers::gnss::Ins::CONVERGING:
        ins_status_.set_type(apollo::drivers::gnss::InsStatus::CONVERGING);
        break;

      case apollo::drivers::gnss::Ins::INVALID:
      default:
        ins_status_.set_type(apollo::drivers::gnss::InsStatus::INVALID);
        break;
    }

    common::util::FillHeader("gnss", &ins_status_);
    insstatus_writer_->Write(ins_status_);
  }
}

void DataParser::CheckGnssStatus(::apollo::drivers::gnss::Gnss *gnss) {
  gnss_status_.set_solution_status(
      static_cast<uint32_t>(gnss->solution_status()));
  gnss_status_.set_num_sats(static_cast<uint32_t>(gnss->num_sats()));
  gnss_status_.set_position_type(static_cast<uint32_t>(gnss->position_type()));

  if (static_cast<uint32_t>(gnss->solution_status()) == 0) {
    gnss_status_.set_solution_completed(true);
  } else {
    gnss_status_.set_solution_completed(false);
  }
  common::util::FillHeader("gnss", &gnss_status_);
  gnssstatus_writer_->Write(gnss_status_);
}

void DataParser::DispatchMessage(Parser::MessageType type, MessagePtr message) {
  switch (type) {
    case Parser::MessageType::GNSS:
      CheckGnssStatus(As<::apollo::drivers::gnss::Gnss>(message));
      break;

    case Parser::MessageType::BEST_GNSS_POS:
      PublishBestpos(message);
      break;

    case Parser::MessageType::IMU:
      PublishImu(message);
      break;

    case Parser::MessageType::INS:
      CheckInsStatus(As<::apollo::drivers::gnss::Ins>(message));
      PublishCorrimu(message);
      PublishOdometry(message);
      break;

    case Parser::MessageType::INS_STAT:
      PublishInsStat(message);
      break;

    case Parser::MessageType::BDSEPHEMERIDES:
    case Parser::MessageType::GPSEPHEMERIDES:
    case Parser::MessageType::GLOEPHEMERIDES:
      PublishEphemeris(message);
      break;

    case Parser::MessageType::OBSERVATION:
      PublishObservation(message);
      break;

    case Parser::MessageType::HEADING:
      PublishHeading(message);
      break;

    case Parser::MessageType::GKV_NAV:
      PublishGkvNav(message);
      break;

    default:
      break;
  }
}

void DataParser::PublishInsStat(const MessagePtr message) {
  auto ins_stat = std::make_shared<InsStat>(*As<InsStat>(message));
  common::util::FillHeader("gnss", ins_stat.get());
  insstat_writer_->Write(ins_stat);
}

void DataParser::PublishBestpos(const MessagePtr message) {
  auto bestpos = std::make_shared<GnssBestPose>(*As<GnssBestPose>(message));
  common::util::FillHeader("gnss", bestpos.get());
  gnssbestpose_writer_->Write(bestpos);
}

void DataParser::PublishImu(const MessagePtr message) {
  auto raw_imu = std::make_shared<Imu>(*As<Imu>(message));
  Imu *imu = As<Imu>(message);

  raw_imu->mutable_linear_acceleration()->set_x(
      -imu->linear_acceleration().y());
  raw_imu->mutable_linear_acceleration()->set_y(imu->linear_acceleration().x());
  raw_imu->mutable_linear_acceleration()->set_z(imu->linear_acceleration().z());

  raw_imu->mutable_angular_velocity()->set_x(-imu->angular_velocity().y());
  raw_imu->mutable_angular_velocity()->set_y(imu->angular_velocity().x());
  raw_imu->mutable_angular_velocity()->set_z(imu->angular_velocity().z());

  common::util::FillHeader("gnss", raw_imu.get());
  rawimu_writer_->Write(raw_imu);
}

void DataParser::PublishOdometry(const MessagePtr message) {
  Ins *ins = As<Ins>(message);
  auto gps = std::make_shared<Gps>();

  double unix_sec = apollo::drivers::util::gps2unix(ins->measurement_time());
  gps->mutable_header()->set_timestamp_sec(unix_sec);
  auto *gps_msg = gps->mutable_localization();

  // 1. pose xyz
  double x = ins->position().lon();
  double y = ins->position().lat();
  x *= DEG_TO_RAD_LOCAL;
  y *= DEG_TO_RAD_LOCAL;

  pj_transform(wgs84pj_source_, utm_target_, 1, 1, &x, &y, NULL);

  gps_msg->mutable_position()->set_x(x);
  gps_msg->mutable_position()->set_y(y);
  gps_msg->mutable_position()->set_z(ins->position().height());

  // 2. orientation
  Eigen::Quaterniond q =
      Eigen::AngleAxisd(ins->euler_angles().z() - 90 * DEG_TO_RAD_LOCAL,
                        Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-ins->euler_angles().y(), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(ins->euler_angles().x(), Eigen::Vector3d::UnitY());

  gps_msg->mutable_orientation()->set_qx(q.x());
  gps_msg->mutable_orientation()->set_qy(q.y());
  gps_msg->mutable_orientation()->set_qz(q.z());
  gps_msg->mutable_orientation()->set_qw(q.w());

  gps_msg->mutable_linear_velocity()->set_x(ins->linear_velocity().x());
  gps_msg->mutable_linear_velocity()->set_y(ins->linear_velocity().y());
  gps_msg->mutable_linear_velocity()->set_z(ins->linear_velocity().z());

  gps_writer_->Write(gps);
  if (config_.tf().enable()) {
    TransformStamped transform;
    GpsToTransformStamped(gps, &transform);
    tf_broadcaster_.SendTransform(transform);
  }
}

void DataParser::PublishCorrimu(const MessagePtr message) {
  Ins *ins = As<Ins>(message);
  auto imu = std::make_shared<CorrectedImu>();
  double unix_sec = apollo::drivers::util::gps2unix(ins->measurement_time());
  imu->mutable_header()->set_timestamp_sec(unix_sec);

  auto *imu_msg = imu->mutable_imu();
  imu_msg->mutable_linear_acceleration()->set_x(
      -ins->linear_acceleration().y());
  imu_msg->mutable_linear_acceleration()->set_y(ins->linear_acceleration().x());
  imu_msg->mutable_linear_acceleration()->set_z(ins->linear_acceleration().z());

  imu_msg->mutable_angular_velocity()->set_x(-ins->angular_velocity().y());
  imu_msg->mutable_angular_velocity()->set_y(ins->angular_velocity().x());
  imu_msg->mutable_angular_velocity()->set_z(ins->angular_velocity().z());

  imu_msg->mutable_euler_angles()->set_x(ins->euler_angles().x());
  imu_msg->mutable_euler_angles()->set_y(-ins->euler_angles().y());
  imu_msg->mutable_euler_angles()->set_z(ins->euler_angles().z() -
                                         90 * DEG_TO_RAD_LOCAL);

  corrimu_writer_->Write(imu);
}

void DataParser::PublishEphemeris(const MessagePtr message) {
  auto eph = std::make_shared<GnssEphemeris>(*As<GnssEphemeris>(message));
  gnssephemeris_writer_->Write(eph);
}

void DataParser::PublishObservation(const MessagePtr message) {
  auto observation =
      std::make_shared<EpochObservation>(*As<EpochObservation>(message));
  epochobservation_writer_->Write(observation);
}

void DataParser::PublishHeading(const MessagePtr message) {
  auto heading = std::make_shared<Heading>(*As<Heading>(message));
  heading_writer_->Write(heading);
}

void DataParser::GpsToTransformStamped(const std::shared_ptr<Gps> &gps,
                                       TransformStamped *transform) {
  transform->mutable_header()->set_timestamp_sec(gps->header().timestamp_sec());
  transform->mutable_header()->set_frame_id(config_.tf().frame_id());
  transform->set_child_frame_id(config_.tf().child_frame_id());
  auto translation = transform->mutable_transform()->mutable_translation();
  translation->set_x(gps->localization().position().x());
  translation->set_y(gps->localization().position().y());
  translation->set_z(gps->localization().position().z());
  auto rotation = transform->mutable_transform()->mutable_rotation();
  rotation->set_qx(gps->localization().orientation().qx());
  rotation->set_qy(gps->localization().orientation().qy());
  rotation->set_qz(gps->localization().orientation().qz());
  rotation->set_qw(gps->localization().orientation().qw());
}

std::string DataParser::PrepareGPRMC(uint64_t utc_time_ns, double lat_deg, double lon_deg, bool pos_valid, bool zero_skipped) {
  // https://gist.github.com/tomfanning/60f94e547c979907e32030c9df7f1272

  std::string result = "$GPRMC";
  char buff[32];

  // utc time
  result += ",";
  auto len = util::time_ns_to_str_hhmmss_msms(utc_time_ns, buff, sizeof(buff));
  if (len > 0) {
    result += std::string(buff);
  }
  else {
    AWARN << "Failed to create GPRMC utc time";
  }

  // positioning state
  result += ",";
  result += pos_valid ? "A" : "V";

  // latitude
  result += ",";
  len = util::lat_deg_to_str_ddmm_mmmm(lat_deg, buff, sizeof(buff));
  if (len > 0) {
    result += std::string(buff);
  }
  else {
    AWARN << "Failed to create GPRMC latitude string";
  }

  // latitude hemisphere
  result += ",";
  result += lat_deg >= 0.0 ? "N" : "S";

  // longitude
  result += ",";
  len = util::lon_deg_to_str_dddmm_mmmm(lon_deg, buff, sizeof(buff));
  if (len > 0) {
    result += std::string(buff);
  }
  else {
    AWARN << "Failed to create GPRMC longitude string";
  }

  // longitude hemisphere
  result += ",";
  result += lat_deg >= 0.0 ? "E" : "W";

  // ground speed (skipped)
  result += ",";
  if (zero_skipped) {
    result += "0.0";
  }

  // ground direction (skipped)
  result += ",";
  if (zero_skipped) {
    result += "0.0";
  }

  // utc date
  result += ",";
  len = util::time_ns_to_str_ddmmyy(utc_time_ns, buff, sizeof(buff));
  if (len > 0) {
    result += std::string(buff);
  }
  else {
    AWARN << "Failed to create GPRMC utc date";
  }

  // magnetic declination (skipped)
  result += ",";
  if (zero_skipped) {
    result += "0.0";
  }

  // direction of magnetic declination (skipped)
  result += ",";
  if (zero_skipped) {
    result += "E";
  }

  // mode indication (skipped)
  result += ",";
  if (zero_skipped) {
    result += "A";
  }

  // crc
  uint8_t crc_sum = 0;
  for (size_t i = 1; i < result.size(); ++i) {
    crc_sum ^= (uint8_t)result[i];
  }
  result += "*";
  result += uint8_to_hex_string(&crc_sum, 1);

  // crcf
  result += '\r';
  result += '\n';

  return result;
}

void DataParser::PublishGkvNav(const MessagePtr message) {
  auto *gkv_nav = As<GkvNav>(message);
  bool pos_valid = false;

  // Pbulish InsStat
  auto ins_stat = std::make_shared<InsStat>();
  ins_stat->set_ins_status(0);
  switch (gkv_nav->alg_stage()) {
    case 50: // GOOD
      pos_valid = true;
      ins_stat->set_pos_type(drivers::gnss::SolutionType::INS_RTKFIXED);
      break;
    default: // INVALID
      ins_stat->set_pos_type(drivers::gnss::SolutionType::NONE);
      break;
  }
  common::util::FillHeader("gnss", ins_stat.get());
  insstat_writer_->Write(ins_stat);

  // Publish CorrectedImu
  auto imu = std::make_shared<CorrectedImu>();
  // double gnss_time_utc = apollo::drivers::util::gps2unix(gkv_nav->measurement_time()); // apollo::cyber::Time::Now().ToSecond();
  double gnss_time_utc = apollo::cyber::Time::Now().ToSecond();
  imu->mutable_header()->set_timestamp_sec(gnss_time_utc);

  auto *imu_msg = imu->mutable_imu();
  ned_to_enu(gkv_nav->linear_acceleration().x(),
             gkv_nav->linear_acceleration().y(),
             gkv_nav->linear_acceleration().z(),
             imu_msg->mutable_linear_acceleration());

  ned_to_enu(gkv_nav->angular_velocity().x(),
             gkv_nav->angular_velocity().y(),
             gkv_nav->angular_velocity().z(),
             imu_msg->mutable_angular_velocity());

  /// NOTE: Must be: (East/North/Up), direction of rotation follows the right-hand rule
  apollo::common::Point3D euler_angles_enu;
  ned_to_enu(gkv_nav->euler_angles().x(),
             gkv_nav->euler_angles().y(),
             gkv_nav->euler_angles().z(),
             &euler_angles_enu);

  imu_msg->mutable_euler_angles()->set_x(euler_angles_enu.x());
  imu_msg->mutable_euler_angles()->set_y(euler_angles_enu.y());
  imu_msg->mutable_euler_angles()->set_z(euler_angles_enu.z());

  corrimu_writer_->Write(imu);

  // Publish Gps
  auto gps = std::make_shared<Gps>();
  gps->mutable_header()->set_timestamp_sec(gnss_time_utc);
  auto *gps_msg = gps->mutable_localization();

  // 1. pose xyz
  double x = gkv_nav->position().lon();
  double y = gkv_nav->position().lat();
  x *= DEG_TO_RAD_LOCAL;
  y *= DEG_TO_RAD_LOCAL;

  pj_transform(wgs84pj_source_, utm_target_, 1, 1, &x, &y, NULL);

  gps_msg->mutable_position()->set_x(x);
  gps_msg->mutable_position()->set_y(y);
  gps_msg->mutable_position()->set_z(gkv_nav->position().height());

  // 2. orientation
  Eigen::Quaterniond q =
      Eigen::AngleAxisd(euler_angles_enu.z(), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(euler_angles_enu.y(), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(euler_angles_enu.x(), Eigen::Vector3d::UnitY());

  gps_msg->mutable_orientation()->set_qx(q.x());
  gps_msg->mutable_orientation()->set_qy(q.y());
  gps_msg->mutable_orientation()->set_qz(q.z());
  gps_msg->mutable_orientation()->set_qw(q.w());

  ned_to_enu(gkv_nav->linear_velocity().x(),
             gkv_nav->linear_velocity().y(),
             gkv_nav->linear_velocity().z(),
             gps_msg->mutable_linear_velocity());

  gps_writer_->Write(gps);
  if (config_.tf().enable()) {
    TransformStamped transform;
    GpsToTransformStamped(gps, &transform);
    tf_broadcaster_.SendTransform(transform);
  }

  // Construct GPRMC message
  const auto gprmc_time_ns = pos_valid ? cyber::Time(gnss_time_utc).ToNanosecond() : cyber::Time::Now().ToNanosecond();
  const auto gprmc_str = PrepareGPRMC(gprmc_time_ns,
                                      gkv_nav->position().lat(),
                                      gkv_nav->position().lon(),
                                      pos_valid,
                                      false);
  // {
  //   std::lock_guard<std::mutex> gprmc_lock(gprmc_str_mutex_);
  //   last_gprmc_str_ = gprmc_str;
  // }
  last_gprmc_str_ = gprmc_str;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
