#include "modules/canbus_vehicle/gazelle/gazelle_vehicle_factory.h"

#define HTTP_IMPLEMENTATION
#include "modules/canbus_vehicle/gazelle/http.h"

#include "cyber/common/log.h"
// #include "modules/canbus/common/canbus_gflags.h"
// #include "modules/common/adapters/adapter_gflags.h"
#include "google/protobuf/util/json_util.h"

using apollo::common::ErrorCode;
using apollo::control::ControlCommand;
using google::protobuf::util::MessageToJsonString;
using google::protobuf::util::JsonStringToMessage;
using google::protobuf::util::JsonPrintOptions;

namespace apollo {
namespace canbus {

bool GazelleVehicleFactory::Init(const CanbusConf *canbus_conf) {
  if (canbus_conf->has_http_client_parameter()) {
    const auto http_client_params = canbus_conf->http_client_parameter();
    if (http_client_params.has_get_url()) {
      get_url = http_client_params.get_url();  
    }
    if (http_client_params.has_post_url()) {
      post_url = http_client_params.post_url();
    }
    if (http_client_params.has_timeout_ms()) {
      http_timeout_ms = http_client_params.timeout_ms();
    }
  }

  AINFO << "get_url inited to: " << get_url.c_str();
  AINFO << "post_url inited to: " << post_url.c_str();
  AINFO << "http_timeout_ms inited to: " << http_timeout_ms;

  AINFO << "Gazelle inited [debug msg]";
  return true;
}

bool GazelleVehicleFactory::Start() {
  AINFO << "Gazelle started [debug msg]";
  return true;
}

void GazelleVehicleFactory::Stop() {
  AINFO << "Gazelle stopped [debug msg]";
}

void GazelleVehicleFactory::UpdateCommand(
    const apollo::control::ControlCommand *control_command) {
  // convert to JSON

  std::string data;

  JsonPrintOptions json_opts;
  json_opts.preserve_proto_field_names = true;

  apollo::control::ControlCommand debug_control_command;
  // debug_control_command.set_throttle(control_command->throttle());
  // debug_control_command.set_brake(control_command->brake());
  // debug_control_command.set_gear_location(control_command->gear_location());
  // debug_control_command.set_steering_target(control_command->steering_target());
  // debug_control_command.mutable_header()->CopyFrom(control_command->header());
  // debug_control_command.mutable_debug()->CopyFrom(control_command->debug());
  // debug_control_command.set_steering_rate(control_command->steering_rate());
  // debug_control_command.mutable_signal()->CopyFrom(control_command->signal());
  // // debug_control_command.mutable_latency_stats()->CopyFrom(control_command->latency_stats());
  // debug_control_command.mutable_engage_advice()->CopyFrom(control_command->engage_advice());

  debug_control_command.CopyFrom(*control_command);
  debug_control_command.clear_latency_stats();
  // debug_control_command.clear_brake();
  
  // auto res = MessageToJsonString(*control_command, &data, json_opts);
  auto res = MessageToJsonString(debug_control_command, &data, json_opts);
  if (!res.ok())
  {
    AERROR << "Failed to convert ControlCommand PROTO to JSON. Status: " << res.ToString();
    return;
  }

  AINFO << data.c_str();

  if (data.empty())
  {
    AINFO << "Empty ControlCommand message to POST. Skip it.";
    return;
  }

  // prepare request
  http_t* request = http_post(post_url.c_str(), data.data(), data.size(), NULL);
  if(!request)
	{
    AERROR_EVERY(100) << "Invalid HTTP connection or POST request: " << post_url.c_str();
    return;
	}

  // process request
  int prev_size = 0;
	http_status_t status = HTTP_STATUS_PENDING;
  auto t_start = std::chrono::steady_clock::now();

	while(status == HTTP_STATUS_PENDING)
	{
		status = http_process(request);
		if(prev_size != (int)request->response_size)
		{
      AINFO << "HTTP POST received byte(s): " << std::to_string((int)request->response_size);
      prev_size = (int)request->response_size;
		}

    const auto t_total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t_start).count();
    if (t_total_ms >= http_timeout_ms)
    {
      AERROR << "HTTP POST request timeout [" << t_total_ms << "/" << http_timeout_ms << "ms]. Abort request.";
      http_release(request);
      return;
    }
	}

  // process result
	if(status == HTTP_STATUS_FAILED)
	{
    AERROR << "HTTP POST failed (" << std::to_string(request->status_code) << "): " << request->reason_phrase;
		http_release(request);
    return;
	}

  const std::string resp_data = (char const*)request->response_data;
  AINFO << "HTTP POST received: " << std::to_string(request->status_code) << " " << request->reason_phrase << "\n" << resp_data.c_str();
  http_release(request);
}

Chassis GazelleVehicleFactory::publish_chassis() {
  Chassis chassis;

  /// TODO: stub. remove later.
  // ControlCommand cmd;

  // apollo::canbus::Chassis_GearPosition gear_pose = Chassis_GearPosition_GEAR_PARKING;
  // gear_counter += 1;

  // if(gear_counter > 1000) {
  //   gear_pose = Chassis_GearPosition_GEAR_PARKING;
  // }
  // else if (gear_counter > 32) {
  //   gear_pose = Chassis_GearPosition_GEAR_DRIVE;
  // }
  // else {
  //   gear_pose = Chassis_GearPosition_GEAR_PARKING;
  // }

  // AINFO << "gear counter=" << gear_counter;
  // AINFO << "gear pose=" << (int)gear_pose;

  // if (gear_pose == Chassis_GearPosition_GEAR_DRIVE) {
  //   if(abs(throttle - 0.0) <= 0.1) {
  //     throttle_delta = 0.2;
  //   }
  //   if(abs(throttle - 40.0) <= 0.1) {
  //     throttle_delta = -0.2;
  //   }
  //   throttle += throttle_delta;
  // }
  // else {
  //   throttle = 0.0;
  // }
  // AINFO << "throttle=" << throttle;

  // if (gear_pose == Chassis_GearPosition_GEAR_DRIVE) {
  //   if(abs(steering_target + 50.0) <= 3.0) { // -(-50)
  //     steering_target_delta = 2.5;
  //   }
  //   if(abs(steering_target - 50.0) <= 3.0) {
  //     steering_target_delta = -2.5;
  //   }
  //   steering_target += steering_target_delta;
  // }
  // else {
  //   steering_target = 0.0;
  // }
  // AINFO << "steering_target=" << steering_target;

  // if (gear_pose == Chassis_GearPosition_GEAR_DRIVE) {
  //   if(gear_counter > 100) {
  //     brake += brake_delta;
  //     if (brake > 50.0) {
  //       brake = 50.0;
  //     }
  //   }
  // }
  // else {
  //   brake = 0.0;
  // }
  // AINFO << "brake=" << brake;

  // cmd.set_throttle(throttle);
  // cmd.set_brake(brake); // 0.0
  // cmd.set_steering_rate(5.0);
  // cmd.set_steering_target(steering_target); // 0.0
  // cmd.set_gear_location(gear_pose); //  Chassis_GearPosition_GEAR_DRIVE
  
  // UpdateCommand(&cmd);
  // return chassis;

  /// TODO: if http failed ?
  // chassis.set_error_code(Chassis::ErrorCode::Chassis_ErrorCode_CHASSIS_ERROR);

  // prepare request
  http_t* request = http_get(get_url.c_str(), NULL);
  if(!request)
	{
    AERROR_EVERY(100) << "Invalid HTTP connection or GET request: " << get_url.c_str();
    /// TODO: what to do here? chassis_.set_error_code ?
    return chassis;
	}

  // process request
  int prev_size = 0;
	http_status_t status = HTTP_STATUS_PENDING;
  auto t_start = std::chrono::steady_clock::now();

	while(status == HTTP_STATUS_PENDING)
	{
		status = http_process(request);
		if(prev_size != (int)request->response_size)
		{
      AINFO << "HTTP GET received byte(s): " << std::to_string((int)request->response_size);
      prev_size = (int)request->response_size;
		}

    const auto t_total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t_start).count();
    if (t_total_ms >= http_timeout_ms)
    {
      AERROR << "HTTP GET request timeout [" << t_total_ms << "/" << http_timeout_ms << "ms]. Abort request.";
      http_release(request);
      /// TODO: what to do here? chassis_.set_error_code ?
      return chassis;
    }
	}

  // process result
	if(status == HTTP_STATUS_FAILED)
	{
    AERROR << "HTTP GET failed (" << std::to_string(request->status_code) << "): " << request->reason_phrase;
		http_release(request);
    /// TODO: what to do here? chassis_.set_error_code ?
    return chassis;
	}

  const std::string data = (char const*)request->response_data;
  AINFO << "HTTP GET received: \n" << data.c_str();
	http_release(request);

  auto res = JsonStringToMessage(data, &chassis);
  if (!res.ok())
  {
    AERROR << "Failed to parse JSON to Chassis PROTO msg. Status: " << res.ToString();
    /// TODO: what to do here? chassis_.set_error_code ?
    return chassis;
  }

  ADEBUG << chassis.ShortDebugString();
  return chassis;
}

void GazelleVehicleFactory::PublishChassisDetail() {
  /// TODO: we dont use chassis_detail ?
  // chassis_detail_writer_->Write(chassis_detail);
}

}  // namespace canbus
}  // namespace apollo