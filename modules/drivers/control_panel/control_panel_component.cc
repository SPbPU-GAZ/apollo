#include "modules/drivers/control_panel/control_panel_component.h"

namespace apollo {
namespace drivers {
namespace control_panel {

ControlPanelComponent::ControlPanelComponent()
    : monitor_logger_buffer_(
          apollo::common::monitor::MonitorMessageItem::CONTROL_PANEL) {}

ControlPanelComponent::~ControlPanelComponent() {
  if (stream_) {
    if (stream_->get_status() == Stream::Status::CONNECTED) {
      if (!stream_->Disconnect()) {
        AERROR << "stream disconnect failed.";
      }
    }
  }

  if (polling_thread_ && polling_thread_->joinable()) {
    AINFO << "Joining polling thread...";
    polling_thread_->join();
    AINFO << "polling thread joined.";
  }
}

bool ControlPanelComponent::Init() {
  // read config
  config::Config control_panel_config;
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_, &control_panel_config)) {
    monitor_logger_buffer_.ERROR("Unable to load control_panel conf file: " +
                                 config_file_path_);
    return false;
  }
  AINFO << "Control panel config: " << control_panel_config.DebugString();
  
  // create stream
  Stream *s = nullptr;
  if (!control_panel_config.has_device_stream()) {
    AERROR << "Error: Config file must provide the device stream.";
    return false;
  }

  const auto& device_stream_conf = control_panel_config.device_stream();
  if (!device_stream_conf.has_device()) {
    AERROR << "Error: Config file must provide the device stream device.";
    return false;
  }
  if (!device_stream_conf.has_baud_rate()) {
    AERROR << "Error: Config file must provide the device stream baudrate.";
    return false;
  }

  s = Stream::create_serial(control_panel_config.device_stream().device().c_str(),
                            control_panel_config.device_stream().baud_rate());
  if (s == nullptr) {
    AERROR << "Failed to create stream.";
    return false;
  }
  stream_.reset(s);

  // connect to device
  if (stream_->get_status() != Stream::Status::CONNECTED) {
      if (!stream_->Connect()) {
        AERROR << "stream connect failed.";
        return false;
      }
  }

  // create writer
  pad_writer_ = node_->CreateWriter<PadMessage>(control_panel_config.planning_pad_topic());

  // create thread
  polling_thread_ = std::make_unique<std::thread>(&ControlPanelComponent::dataPoll, this);

  return true;
}

void ControlPanelComponent::dataPoll() {
  while (apollo::cyber::OK()) {
    if (!stream_) {
      AERROR_EVERY(1000) << "Stream object is nullptr! Abort polling.";
      return;
    }

    size_t length = stream_->read(buffer_, BUFFER_SIZE);
    if (length > 0) {
      const auto cmd = std::string(reinterpret_cast<const char *>(buffer_));

      PadMessage pad;
      if (cmd == std::string("PAUSE!\r\n")) {
        AINFO << "Received PAUSE command, push STOP pad message.";
        pad.set_action(apollo::planning::PadMessage_DrivingAction_STOP);
        apollo::common::util::FillHeader("control_panel", &pad);
        pad_writer_->Write(pad);
      }
      else if (cmd == std::string("DRIVE!\r\n")) {
        AINFO << "Received DRIVE command, push FOLLOW pad message.";
        pad.set_action(apollo::planning::PadMessage_DrivingAction_FOLLOW);
        apollo::common::util::FillHeader("control_panel", &pad);
        pad_writer_->Write(pad);
      }
      else if (cmd == std::string("STOP!\r\n\n")) {
        AINFO << "Received STOP command, push STOP pad message.";
        pad.set_action(apollo::planning::PadMessage_DrivingAction_STOP);
        apollo::common::util::FillHeader("control_panel", &pad);
        pad_writer_->Write(pad);
      }
      else {
        AWARN << "Received unknown command: " << cmd.c_str() << ". Nothing to push.";
      }
    }
  }

  AINFO << "dataPoll finished";
}

}  // namespace control_panel
}  // namespace drivers
}  // namespace apollo
