/**
 * @file gazelle_vehicle_factory.h
 */

#pragma once

#include <memory>

#include "modules/canbus/proto/canbus_conf.pb.h"
// #include "modules/canbus/proto/vehicle_parameter.pb.h"
// #include "modules/canbus_vehicle/lincoln/proto/lincoln.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"

#include "cyber/cyber.h"
#include "modules/canbus/vehicle/abstract_vehicle_factory.h"
#include "modules/common/status/status.h"

/**
 * @namespace apollo::canbus
 * @brief apollo::canbus
 */
namespace apollo {
namespace canbus {

/**
 * @class GazelleVehicleFactory
 *
 * @brief this class is inherited from AbstractVehicleFactory.
 */
class GazelleVehicleFactory : public AbstractVehicleFactory {
 public:
  /**
   * @brief destructor
   */
  virtual ~GazelleVehicleFactory() = default;

  /**
   * @brief init vehicle factory
   * @returns true if successfully initialized
   */
  bool Init(const CanbusConf *canbus_conf) override;

  /**
   * @brief start
   * @returns true if successfully started
   */
  bool Start() override;

  /**
   * @brief stop
   */
  void Stop() override;

  /**
   * @brief update control command
   */
  void UpdateCommand(
      const apollo::control::ControlCommand *control_command) override;

  /**
   * @brief publish chassis messages
   */
  Chassis publish_chassis() override;

  /**
   * @brief publish chassis for vehicle messages
   */
  void PublishChassisDetail() override; // TODO: we don't use this ?

  private:
    std::unique_ptr<::apollo::cyber::Node> node_ = nullptr;
    std::string get_url = "http://192.168.100.100:5000/api/chassis_state";
    std::string post_url = "http://192.168.100.100:5000/api/control_command";
    int64_t http_timeout_ms = 1000;

    // float throttle = 0.0; // TODO: test only
    // float throttle_delta = 0.5; // TODO: test only

    // float steering_target = 0.0; // TODO: test only
    // float steering_target_delta = 2.5; // TODO: test only

    // float brake = 0.0; // TODO: test only
    // float brake_delta = 0.5; // TODO: test only

    // int gear_counter = 0; // TODO: test only

//   std::shared_ptr<::apollo::cyber::Writer<::apollo::canbus::Lincoln>>
//       chassis_detail_writer_;

};

CYBER_REGISTER_VEHICLEFACTORY(GazelleVehicleFactory)

}  // namespace canbus
}  // namespace apollo
