#include <fstream>
#include "cyber/cyber.h"
#include "modules/drivers/gnss/stream/stream.h"

namespace apollo {
namespace drivers {
namespace gnss {

class FileStream : public Stream {
 public:
  FileStream(const std::string& file_path);
  ~FileStream();
  virtual bool Connect();
  virtual bool Disconnect();
  virtual size_t read(uint8_t* buffer, size_t max_length);
  virtual size_t write(const uint8_t* data, size_t length);

 private:
  FileStream();
  void open();
  void close();

 private:
  std::string file_path_;
  std::ifstream ifs_;
};

Stream* Stream::create_file(const std::string& file_path) {
  return new FileStream(file_path);
}

FileStream::FileStream(const std::string& file_path) {
  file_path_ = file_path;
  if (file_path.empty()) {
    status_ = Stream::Status::ERROR;
  }
}

FileStream::~FileStream() {
  this->close();
}

void FileStream::open(void) {
  ifs_.open(file_path_, std::ifstream::in);
  if (!ifs_.is_open()) {
    AERROR << "Failed to open stream file: " << file_path_.c_str();
  }

  AINFO << "File stream openned: " << file_path_.c_str();
}

void FileStream::close(void) {
  if (ifs_.is_open()) {
    ifs_.close();
    status_ = Stream::Status::DISCONNECTED;
  }

  AINFO << "File stream closed: " << file_path_.c_str();
}

bool FileStream::Connect() {
  if (!ifs_.is_open()) {
    this->open();
    if (!ifs_.is_open()) {
      status_ = Stream::Status::ERROR;
      return false;
    }
  }

  if (status_ == Stream::Status::CONNECTED) {
    return true;
  }

  status_ = Stream::Status::CONNECTED;
  return true;
}

bool FileStream::Disconnect() {
  if (!ifs_.is_open()) {
    // not open
    return false;
  }

  this->close();
  return true;
}

size_t FileStream::read(uint8_t* buffer, size_t max_length) {
  if (!ifs_.is_open()) {
    if (!Connect()) {
      return 0;
    }
    AINFO << "Connect " << file_path_.c_str() << " success.";
  }

  ifs_.read((char* )buffer, max_length);
  if (ifs_.bad()) {
    AERROR << "Failed to read from file stream: " << file_path_.c_str();
    return 0;
  }

  return ifs_.gcount();
}

size_t FileStream::write(const uint8_t* data, size_t length) {
  AERROR << "Writing to file stream unsupported";
  return 0;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
