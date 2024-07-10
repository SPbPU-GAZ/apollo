#pragma once

#include <memory>
#include <thread>

#include "cyber/cyber.h"
#include "cyber/component/timer_component.h"
#include "cyber/time/time.h"

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/telemetry/proto/config.pb.h"
#include "modules/telemetry/proto/telemetry.pb.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/basic_msgs/vehicle_signal.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/localization/msf/common/util/frame_transform.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/telemetry/output.h"
#include "modules/common/math/box2d.h"
// #include "modules/common_msgs/planning_msgs/pad_msg.pb.h"

namespace apollo {
namespace telemetry {

using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::cyber::TimerComponent;
using apollo::canbus::Chassis;
using apollo::perception::PerceptionObstacles;
using apollo::perception::PerceptionObstacle;
using apollo::telemetry::config::Config;
using apollo::telemetry::packet::Packet;
using apollo::telemetry::packet::ObjData;
using apollo::common::VehicleSignal;
using apollo::localization::msf::FrameTransform;
using apollo::localization::msf::WGS84Corr;
using apollo::localization::msf::UTMCoor;
using apollo::localization::LocalizationEstimate;
using apollo::common::VehicleConfigHelper;
using apollo::hdmap::HDMapUtil;
using apollo::common::PathPoint;
// using apollo::planning::PadMessage;
using apollo::telemetry::packet::ObstacleOnTheWay;
using apollo::telemetry::config::Config_Lane;

class TelemetryComponent : public TimerComponent {
 public:
  TelemetryComponent();
  virtual ~TelemetryComponent();
  bool Init() override;
  bool Proc() override;

 private:
  void ProduceTelemetryPacket(Packet *packet,
                              const std::shared_ptr<LocalizationEstimate> &localization_estimate,
                              const std::shared_ptr<Chassis> &chassis,
                              const std::shared_ptr<PerceptionObstacles> &perception_obstacles);
  bool CalculateObjectData(ObjData* obj_data,
                           const LocalizationEstimate& localization_estimate,
                           const PerceptionObstacle& obstacle,
                           bool* obstacle_on_the_way);
  common::math::Box2d GetVehicleObstacleZone(const common::PathPoint &path_point,
                                            double zone_front_edge_to_center,
                                            double zone_rear_edge_to_center,
                                            double zone_width);

 private:
   int utm_zone_ = 0;
   bool convert_to_wgs84_ = true;
   bool json_add_whitespace_ = false;
   bool steering_rotation_to_deg_ = true;

   bool obstacle_zone_enable_ = false;
   double obstacle_zone_front_edge_to_center_ = 10.0;
   double obstacle_zone_rear_edge_to_center_ = 1.5;
   double obstacle_zone_width_ = 3.0;

   size_t sequence_num_  = 0;
   std::unique_ptr<OutputSocket> output_socket_;
   std::shared_ptr<cyber::Reader<Chassis>> chassis_reader_;
   std::shared_ptr<cyber::Reader<PerceptionObstacles>> perception_obstacles_reader_;
   std::shared_ptr<cyber::Reader<LocalizationEstimate>> localization_estimate_reader_;
   apollo::common::monitor::MonitorLogBuffer monitor_logger_buffer_;
   std::shared_ptr<Writer<ObstacleOnTheWay>> obstacle_on_the_way_writer_;

   Config telemetry_config_;
};

CYBER_REGISTER_COMPONENT(TelemetryComponent)

}  // namespace telemetry
}  // namespace apollo
