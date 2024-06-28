#pragma once

#include <memory>
#include <thread>

#include "cyber/cyber.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/drivers/control_panel/proto/config.pb.h"
#include "modules/common_msgs/planning_msgs/pad_msg.pb.h"
#include "modules/drivers/control_panel/stream/stream.h"

namespace apollo {
namespace drivers {
namespace control_panel {

using apollo::cyber::Writer;
using apollo::cyber::Component;
using apollo::planning::PadMessage;
using apollo::drivers::control_panel::config::Config;

class ControlPanelComponent : public Component<> {
 public:
  ControlPanelComponent();
  virtual ~ControlPanelComponent();
  bool Init() override;

 private:
  void dataPoll();

 private:
   static constexpr size_t BUFFER_SIZE = 2048;
   uint8_t buffer_[BUFFER_SIZE] = {0};

   std::shared_ptr<Stream> stream_;
   std::unique_ptr<std::thread> polling_thread_;
   std::shared_ptr<Writer<PadMessage>> pad_writer_;
   apollo::common::monitor::MonitorLogBuffer monitor_logger_buffer_;
};

CYBER_REGISTER_COMPONENT(ControlPanelComponent)

}  // namespace control_panel
}  // namespace drivers
}  // namespace apollo
