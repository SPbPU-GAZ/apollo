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

  apollo::control::ControlCommand tuned_control_command;
  tuned_control_command.CopyFrom(*control_command);
  tuned_control_command.clear_latency_stats();

  // to exclude zero value on throttle
  const auto throttle_ = std::max(31.0, control_command->throttle());
  tuned_control_command.set_throttle(throttle_);

  // auto steering_target_ = std::max(control_command->steering_target() - 2.0, -100.0);
  // tuned_control_command.set_steering_target(-steering_target_);

  tuned_control_command.set_steering_target(-control_command->steering_target());

  auto res = MessageToJsonString(tuned_control_command, &data, json_opts);
  if (!res.ok())
  {
    AERROR << "Failed to convert ControlCommand PROTO to JSON. Status: " << res.ToString();
    return;
  }

  // AINFO << data.c_str();

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
    AERROR_EVERY(50) << "HTTP GET failed (" << std::to_string(request->status_code) << "): " << request->reason_phrase;
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

  // Stub data
  chassis.mutable_header()->set_timestamp_sec(apollo::cyber::Time::Now().ToSecond());
  chassis.mutable_header()->set_frame_id("ego_vehicle");
  chassis.set_engine_started(true);
  chassis.set_driving_mode(apollo::canbus::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE);
  chassis.set_steering_percentage(-chassis.steering_percentage());
  chassis.set_steering_percentage_cmd(-chassis.steering_percentage_cmd());

  // if (chassis.steering_percentage() > 0.0) {
  //   chassis.set_steering_percentage(chassis.steering_percentage() / left_steering_gain);
  // }
  // else {
  //   chassis.set_steering_percentage(chassis.steering_percentage());
  // }

  ADEBUG << chassis.ShortDebugString();
  return chassis;
}

void GazelleVehicleFactory::PublishChassisDetail() {
  /// TODO: we dont use chassis_detail ?
  // chassis_detail_writer_->Write(chassis_detail);
}

}  // namespace canbus
}  // namespace apollo