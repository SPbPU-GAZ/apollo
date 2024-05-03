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

namespace apollo {
namespace canbus {

bool GazelleVehicleFactory::Init(const CanbusConf *canbus_conf) {
  AERROR << "Gazelle inited [debug msg]";
  return true;
}

bool GazelleVehicleFactory::Start() {
  AERROR << "Gazelle started [debug msg]";
  return true;
}

void GazelleVehicleFactory::Stop() {
  AERROR << "Gazelle stopped [debug msg]";
}

void GazelleVehicleFactory::UpdateCommand(
    const apollo::control::ControlCommand *control_command) {
  // convert to JSON
  std::string data;
  auto res = MessageToJsonString(*control_command, &data);
  if (!res.ok())
  {
    AERROR << "Failed to convert ControlCommand PROTO to JSON. Status: " << res.ToString();
    return;
  }

  if (data.empty())
  {
    AINFO << "Empty ControlCommand message to POST. Skip it.";
    return;
  }

  // prepare request
  http_t* request = http_post(post_url, data.data(), data.size(), NULL);
  if(!request)
	{
    AERROR_EVERY(100) << "Invalid HTTP connection or POST request: " << post_url;
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

  // /// TODO: stub. remove later.
  // ControlCommand cmd;
  // cmd.set_acceleration(50.0);
  // UpdateCommand(&cmd);
  // return chassis;

  /// TODO: if http failed ?
  // chassis.set_error_code(Chassis::ErrorCode::Chassis_ErrorCode_CHASSIS_ERROR);

  // prepare request
  http_t* request = http_get(get_url, NULL);
  if(!request)
	{
    AERROR_EVERY(100) << "Invalid HTTP connection or GET request: " << get_url;
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