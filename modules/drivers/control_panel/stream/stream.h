#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "cyber/cyber.h"

#include "modules/drivers/control_panel/stream/macros.h"

namespace apollo {
namespace drivers {
namespace control_panel {

class Stream {
 public:
  // Currently the following baud rates are supported:
  //  9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600.
  static Stream *create_serial(const char *device_name, uint32_t baud_rate,
                               uint32_t timeout_usec = 0);
  virtual ~Stream() {}

  // Stream status.
  enum class Status {
    DISCONNECTED,
    CONNECTED,
    ERROR,
  };

  static constexpr size_t NUM_STATUS =
      static_cast<int>(Stream::Status::ERROR) + 1;
  Status get_status() const { return status_; }

  // Returns whether it was successful to connect.
  virtual bool Connect() = 0;

  // Returns whether it was successful to disconnect.
  virtual bool Disconnect() = 0;

  void RegisterLoginData(const std::vector<std::string> login_data) {
    login_data_.assign(login_data.begin(), login_data.end());
  }

  void Login() {
    for (size_t i = 0; i < login_data_.size(); ++i) {
      write(login_data_[i]);
      AINFO << "Login: " << login_data_[i];
      // sleep a little to avoid overrun of the slow serial interface.
      cyber::Duration(0.5).Sleep();
    }
  }

  // Reads up to max_length bytes. Returns actually number of bytes read.
  virtual size_t read(uint8_t *buffer, size_t max_length) = 0;

  // Returns how many bytes it was successful to write.
  virtual size_t write(const uint8_t *buffer, size_t length) = 0;

  size_t write(const std::string &buffer) {
    return write(reinterpret_cast<const uint8_t *>(buffer.data()),
                 buffer.size());
  }

 protected:
  Stream() {}

  Status status_ = Status::DISCONNECTED;

 private:
  std::vector<std::string> login_data_;
  DISABLE_COPY_AND_ASSIGN(Stream);
};

}  // namespace control_panel
}  // namespace drivers
}  // namespace apollo
