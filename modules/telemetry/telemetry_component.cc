#include "modules/telemetry/telemetry_component.h"
#include "google/protobuf/util/json_util.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"

using google::protobuf::util::MessageToJsonString;
using google::protobuf::util::JsonPrintOptions;

namespace apollo {
namespace telemetry {

TelemetryComponent::TelemetryComponent()
    : monitor_logger_buffer_(
          apollo::common::monitor::MonitorMessageItem::TELEMETRY) {}

TelemetryComponent::~TelemetryComponent() {

}

bool TelemetryComponent::Init() {
  // read config
  Config telemetry_config;
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_, &telemetry_config)) {
    monitor_logger_buffer_.ERROR("Unable to load telemetry conf file: " +
                                 config_file_path_);
    return false;
  }
  AINFO << "Telemetry config: " << telemetry_config.DebugString();

  // get utm_zone
  if (telemetry_config.has_utm_zone()) {
    utm_zone_ = telemetry_config.utm_zone();
  }
  else {
    AWARN << "Can't find utm zone in config. Using default: " << utm_zone_;
  }

  // get convert_to_wgs84
  if (telemetry_config.has_convert_to_wgs84()) {
    convert_to_wgs84_ = telemetry_config.convert_to_wgs84();
  }
  else {
    AWARN << "Can't find convert_to_wgs84 flag in config. Using default: " << convert_to_wgs84_;
  }

  // create chassis reader
  if (telemetry_config.has_chassis_topic()) {
    chassis_reader_ = node_->CreateReader<Chassis>(telemetry_config.chassis_topic(), nullptr);
  }
  else {
    AERROR << "Failed to create Chassis reader. Chassis topic not found in config.";
    return false;
  }
  ACHECK(chassis_reader_ != nullptr);

 // create localization estimate reader
  if (telemetry_config.has_localization_topic()) {
    localization_estimate_reader_ = node_->CreateReader<LocalizationEstimate>(telemetry_config.localization_topic(), nullptr);
  }
  else {
    AERROR << "Failed to create LocalizationEstimate reader. LocalizationEstimate topic not found in config.";
    return false;
  }
  ACHECK(localization_estimate_reader_ != nullptr);

  // create perception obstacles reader
  if (telemetry_config.has_perception_obstacles_topic()) {
    perception_obstacles_reader_ = node_->CreateReader<PerceptionObstacles>(telemetry_config.perception_obstacles_topic(), nullptr);    
  }
  else {
    AERROR << "Failed to create PerceptionObstacles reader. PerceptionObstacles topic not found in config.";
    return false;
  }
  ACHECK(perception_obstacles_reader_ != nullptr);

  // create output socket
  if (!telemetry_config.has_server_ip() || !telemetry_config.has_server_port()) {
    AERROR << "Failed to find UDP server_ip/server_port params in config. Please set.";
    return false;
  }

  output_socket_ = std::make_unique<OutputSocket>();
  if (!output_socket_->init(telemetry_config.server_port(), telemetry_config.server_ip())) {
    AERROR << "Failed to create output socket";
    return false;
  }

  // other settings
  if (telemetry_config.has_json_add_whitespace()) {
    json_add_whitespace_ = telemetry_config.json_add_whitespace();
  }
  else {
    AWARN << "Can't find json_add_whitespace flag in config. Using default: " << json_add_whitespace_;
  }

  if (telemetry_config.has_steering_rotation_to_deg()) {
    steering_rotation_to_deg_ = telemetry_config.steering_rotation_to_deg();
  }
  else {
    AWARN << "Can't find steering_rotation_to_deg flag in config. Using default: " << steering_rotation_to_deg_;
  }

  return true;
}

bool TelemetryComponent::CalculateObjectData(ObjData* obj_data,
                                             const LocalizationEstimate& localization_estimate,
                                             const PerceptionObstacle& obstacle) {
  // check inputs
  if (!obstacle.has_position() ||
      !obstacle.has_width() ||
      !obstacle.has_length() ||
      !obstacle.has_theta() ||
      !obstacle.has_type()) {
    AWARN << "position/width/length/theta/type not found in PerceptionObstacle";
    return false;
  }

  if (!localization_estimate.has_pose()) {
    AWARN << "pose not found in LocalizationEstimate";
    return false;  
  }

  const auto& vehicle_pose = localization_estimate.pose();
  if (!vehicle_pose.has_orientation() || !vehicle_pose.has_position()) {
    AWARN << "position/orientation not found in LocalizationEstimate";
    return false;
  }

  // set object type
  switch (obstacle.type()) {
    case PerceptionObstacle::Type::PerceptionObstacle_Type_VEHICLE:
      obj_data->set_objecttype(ObjData::ObjType::ObjData_ObjType_Car);
      break;
    case PerceptionObstacle::Type::PerceptionObstacle_Type_PEDESTRIAN:
      obj_data->set_objecttype(ObjData::ObjType::ObjData_ObjType_Person);
      break;
    default:
      obj_data->set_objecttype(ObjData::ObjType::ObjData_ObjType_Unknown);
      break;
  }

  // get vehicle dimensions
  VehicleConfigHelper::GetConfig();

  // calculate vehicle bbox
  const double vehicle_theta = common::math::QuaternionToHeading(
    vehicle_pose.orientation().qw(), vehicle_pose.orientation().qx(),
    vehicle_pose.orientation().qy(), vehicle_pose.orientation().qz());

  apollo::common::PathPoint vehicle_pp;
  vehicle_pp.set_x(vehicle_pose.position().x());
  vehicle_pp.set_y(vehicle_pose.position().y());
  vehicle_pp.set_theta(vehicle_theta);
  const auto vehicle_bbox = VehicleConfigHelper::GetBoundingBox(vehicle_pp);

  // calculate obstacle bbox
  const common::math::Vec2d obstacle_center(obstacle.position().x(), obstacle.position().y());
  const auto obstacle_bbox = common::math::Box2d(obstacle_center,
                                                 obstacle.theta(),
                                                 obstacle.length(),
                                                 obstacle.width());

  // calculate alpha
  const auto ad_vec = obstacle_bbox.center() - vehicle_bbox.center();
  auto alpha = (vehicle_bbox.heading() - ad_vec.Angle());
  if (alpha < 0.0) {
    alpha += 2 * M_PI;
  }
  obj_data->set_alpha(RAD_TO_DEG * alpha);

  // find Cn index
  size_t obstacle_cn_idx = 0;
  const auto vehicle_corners = vehicle_bbox.GetAllCorners();
  const auto obstacle_corners = obstacle_bbox.GetAllCorners();

  double min_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < vehicle_corners.size(); ++i) {
    for (size_t j = 0; j < obstacle_corners.size(); ++j) {
      const auto distance = vehicle_corners[i].DistanceTo(obstacle_corners[j]);
      if (distance < min_distance) {
        obstacle_cn_idx = j;
        min_distance = distance;
      }
    }
  }
  obj_data->set_bc(min_distance);

  // obstacle Dn, Cn
  std::vector<common::math::Vec2d> obstacle_dn_cn_in_jn_kn;
  obstacle_dn_cn_in_jn_kn.push_back(obstacle_center);
  obstacle_dn_cn_in_jn_kn.push_back(obstacle_corners[obstacle_cn_idx]);

  // obstacle In, Jn, Kn
  int curr_idx = obstacle_cn_idx;
  int cnt = obstacle_corners.size();
  while(--cnt) {
    curr_idx--;
    if (curr_idx < 0) {
      curr_idx = obstacle_corners.size() - 1;
    }
    obstacle_dn_cn_in_jn_kn.push_back(obstacle_corners[curr_idx]);
  }

  // convert to WGS84
  if (convert_to_wgs84_) {
    for (size_t i = 0; i < obstacle_dn_cn_in_jn_kn.size(); ++i) {
      auto& pt = obstacle_dn_cn_in_jn_kn[i];
      WGS84Corr wgs_coords;
      if (FrameTransform::UtmXYToLatlon(pt.x(), pt.y(), utm_zone_, true, &wgs_coords)) {
        pt.set_x(RAD_TO_DEG * wgs_coords.lat);
        pt.set_y(RAD_TO_DEG * wgs_coords.log);
      }
      else {
        AERROR << "Failed to convert obstacle coordinates from UTM to WGS84";
        return false;
      }
    }
  }

  // to protobuf
  assert(obstacle_dn_cn_in_jn_kn.size() == 5);

  const auto& dn = obstacle_dn_cn_in_jn_kn[0];
  obj_data->mutable_coordinates()->add_d(dn.x());
  obj_data->mutable_coordinates()->add_d(dn.y());

  const auto& cn = obstacle_dn_cn_in_jn_kn[1];
  obj_data->mutable_coordinates()->add_c(cn.x());
  obj_data->mutable_coordinates()->add_c(cn.y());

  const auto& in = obstacle_dn_cn_in_jn_kn[2];
  obj_data->mutable_coordinates()->add_i(in.x());
  obj_data->mutable_coordinates()->add_i(in.y());

  const auto& jn = obstacle_dn_cn_in_jn_kn[3];
  obj_data->mutable_coordinates()->add_j(jn.x());
  obj_data->mutable_coordinates()->add_j(jn.y());

  const auto& kn = obstacle_dn_cn_in_jn_kn[4];
  obj_data->mutable_coordinates()->add_k(kn.x());
  obj_data->mutable_coordinates()->add_k(kn.y());

  // get lane id
  // const auto* hdmap = HDMapUtil::BaseMapPtr();
  // if (hdmap) {
  //   common::PointENU pt;
  //   pt.set_x(vehicle_bbox.center_x());
  //   pt.set_y(vehicle_bbox.center_y());

  //   hdmap::LaneInfoConstPtr nearest_lane;
  //   double nearest_s = 0.0;
  //   double nearest_l = 0.0;
  //   int ret = hdmap->GetNearestLane(pt, &nearest_lane, &nearest_s, &nearest_l);
  //   if (ret == 0) {
  //     AINFO << "Found nearest lane with id: " << nearest_lane->id().id();
  //     // AINFO << "Nearest lane road_id: " << nearest_lane->road_id().id();
  //     // AINFO << "Nearest lane section_id: " << nearest_lane->section_id().id();
  //     // AINFO << "Vehicle position: " << pt.x() << ", " << pt.y();
  //     // AINFO << "nearest_s: " << nearest_s << ", nearest_l: " << nearest_l;
  //   }
  //   else {
  //     AWARN << "Failed to find nearest lane. Skip obstacle lane id.";
  //   }
  // }
  // else {
  //   AWARN << "Failed to get base hdmap. Skip obstacle lane id.";
  // }

  return true;
}

void TelemetryComponent::ProduceTelemetryPacket(Packet *packet,
  const std::shared_ptr<LocalizationEstimate> &localization_estimate,
  const std::shared_ptr<Chassis> &chassis,
  const std::shared_ptr<PerceptionObstacles> &perception_obstacles) {

  if (chassis) {
    // TransmissionState
    if (chassis->has_gear_location()) {
        switch (chassis->gear_location()) {
        case Chassis::GearPosition::Chassis_GearPosition_GEAR_DRIVE:
            packet->set_transmissionstate(Packet::TransState::Packet_TransState_D);
            break;
        case Chassis::GearPosition::Chassis_GearPosition_GEAR_NEUTRAL:
            packet->set_transmissionstate(Packet::TransState::Packet_TransState_N);
            break;
        case Chassis::GearPosition::Chassis_GearPosition_GEAR_PARKING:
            packet->set_transmissionstate(Packet::TransState::Packet_TransState_P);
            break;
        case Chassis::GearPosition::Chassis_GearPosition_GEAR_REVERSE:
            packet->set_transmissionstate(Packet::TransState::Packet_TransState_R);
            break;
        default:
            AWARN << "Can't find proper transmission state (D, N, P, R) in Chassis";
            break;
        }
    }
    else {
        AWARN << "gear_location not found in Chassis";
    }

    // SteeringRotation
    if (chassis->has_steering_percentage()) {
      auto value = chassis->steering_percentage();

      if (steering_rotation_to_deg_) {
        const auto& cfg = VehicleConfigHelper::GetConfig();
        if (cfg.has_vehicle_param() && cfg.vehicle_param().has_max_steer_angle()) {
          value = RAD_TO_DEG * (value / 100.0) * cfg.vehicle_param().has_max_steer_angle();
        }
        else {
          AWARN << "Can't find max_steer_angle in vehicle params file";
        }
      }

      packet->set_steeringrotation(value);
    }
    else {
        AWARN << "steering_percentage not found in Chassis";
    }

    // RequestedAcceleratorPower
    if (chassis->has_throttle_percentage()) {
        packet->set_requestedacceleratorpower(chassis->throttle_percentage());
    }
    else {
        AWARN << "throttle_percentage not found in Chassis";
    }

    // RequestedBrakePower
    if (chassis->has_brake_percentage()) {
        packet->set_requestedbrakepower(chassis->brake_percentage());
    }
    else {
        AWARN << "brake_percentage not found in Chassis";
    }

    // Velocity
    if (chassis->has_wheel_speed()) {
        const auto& ws = chassis->wheel_speed();

        if (ws.has_wheel_spd_fl()) {
        packet->mutable_velocity()->set_leftfront(ws.wheel_spd_fl());
        }
        else {
        AWARN << "wheel_spd_fl not found in Chassis";
        }

        if (ws.has_wheel_spd_fr()) {
        packet->mutable_velocity()->set_rightfront(ws.wheel_spd_fr());
        }
        else {
        AWARN << "wheel_spd_fr not found in Chassis";
        }

        if (ws.has_wheel_spd_rl()) {
        packet->mutable_velocity()->set_leftrear(ws.wheel_spd_rl());
        }
        else {
        AWARN << "wheel_spd_rl not found in Chassis";
        }

        if (ws.has_wheel_spd_rr()) {
        packet->mutable_velocity()->set_rightrear(ws.wheel_spd_rr());
        }
        else {
        AWARN << "wheel_spd_rr not found in Chassis";
        }
    }
    else {
        AWARN << "wheel_speed not found in Chassis";
    }

    // TurnSignalState
    if (chassis->has_signal() && chassis->signal().has_turn_signal()) {
        switch (chassis->signal().turn_signal()) {
            case VehicleSignal::TURN_NONE:
            packet->set_turnsignalstate("0");
            break;
            case VehicleSignal::TURN_LEFT:
            packet->set_turnsignalstate("left");
            break;
            case VehicleSignal::TURN_RIGHT:
            packet->set_turnsignalstate("right");
            break;
            case VehicleSignal::TURN_HAZARD_WARNING:
            packet->set_turnsignalstate("left&right");
            break;
            default:
            AWARN << "Can't find proper turn_signal state in Chassis";
            break;       
        }
    }
    else {
        AWARN << "turn_signal not found in Chassis";
    }
  }
  else {
    AERROR << "chassis is nullptr";
  }

  if (localization_estimate && perception_obstacles) {
    // ObjectData
    for (const auto& obstacle : perception_obstacles->perception_obstacle()) {
      auto* obj_data = packet->add_objectdata();
      if (!CalculateObjectData(obj_data, *localization_estimate.get(), obstacle)) {
        AWARN << "Failed to calculate object coordinates";
        packet->mutable_objectdata()->RemoveLast();
        continue;
      }
    }

    // sort objects by alpha value
    std::sort(packet->mutable_objectdata()->begin(), packet->mutable_objectdata()->end(),
      [](const ObjData& a, const ObjData& b) {return a.alpha() < b.alpha();}
    );
  }
  else {
    AERROR << "perception_obstacles or localization_estimate is nullptr";
  }
}

bool TelemetryComponent::Proc() {
  // get localization estimate
  localization_estimate_reader_->Observe();
  const auto &localization_estimate_msg = localization_estimate_reader_->GetLatestObserved();
  if (localization_estimate_msg == nullptr) {
    AERROR << "LocalizationEstimate msg is not ready!";
    return false;
  }

  // get chassis
  chassis_reader_->Observe();
  const auto &chassis_msg = chassis_reader_->GetLatestObserved();
  if (chassis_msg == nullptr) {
    AERROR << "Chassis msg is not ready!";
    return false;
  }

  // get perception obstacles
  perception_obstacles_reader_->Observe();
  const auto &perception_obstacles_msg = perception_obstacles_reader_->GetLatestObserved();
  if (perception_obstacles_msg == nullptr) {
    AERROR << "PerceptionObstacles msg is not ready!";
    return false;
  }

  // produce packet
  auto packet = std::make_shared<Packet>();
  packet->mutable_header()->set_sequence_num(sequence_num_++);
  packet->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  packet->mutable_header()->set_frame_id("telemetry");

  ProduceTelemetryPacket(packet.get(),
                         localization_estimate_msg,
                         chassis_msg,
                         perception_obstacles_msg);

  // convert to json
  JsonPrintOptions json_opts;
  json_opts.preserve_proto_field_names = true;
  json_opts.add_whitespace = json_add_whitespace_;

  std::string data;
  auto res = MessageToJsonString(*packet.get(), &data, json_opts);
  if (!res.ok())
  {
      AERROR << "Failed to convert PROTO to JSON. Status: " << res.ToString();
      return false;
  }

  // AINFO << data.c_str();

  // send over udp
  if (output_socket_) {
    const auto sent_bytes = output_socket_->send_data(data.c_str());
    AINFO << "Sent " << sent_bytes << "/" << data.size() << " bytes";
  }
  else {
    AERROR << "output_socket_ is nullptr";
    return false;
  }

  return true;
}

}  // namespace telemetry
}  // namespace apollo
