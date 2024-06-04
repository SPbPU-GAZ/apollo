#include "modules/drivers/gnss/parser/gkv_messages.h"
#include "modules/drivers/gnss/parser/parser.h"
#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace gnss {

// anonymous namespace that contains helper constants and functions
namespace {
// Maximum possible buffer size (max length of data + length of header + length of CRC)
static const size_t BUFFER_SIZE = 256 + sizeof(gkv::Header) + gkv::CRC_LENGTH;

// CRC lookup table from GKV-10 protocol document
static uint32_t crc32_tab[] = {
  0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
  0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
  0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
  0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
  0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
  0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
  0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
  0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
  0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
  0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
  0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
  0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
  0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
  0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
  0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
  0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
  0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
  0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
  0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
  0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
  0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
  0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
  0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
  0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
  0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
  0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
  0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
  0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
  0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
  0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
  0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
  0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
  0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
  0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
  0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
  0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
  0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
  0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
  0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
  0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
  0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
  0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
  0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

// CRC algorithm from GKV-10 protocol document
uint32_t crc32(uint32_t crc, const void *buf, size_t size) {
  const uint8_t *p = (uint8_t *)buf;
  crc = crc ^ ~0U;
  while (size--) {
    crc = crc32_tab[(crc ^ *p++) & 0xFF] ^ (crc >> 8);
  }

  return crc ^ ~0U;
}

const char* ResponsePacketTypeToString(gkv::ResponsePacketType c) {
  switch(c)
  {
    case gkv::ResponsePacketType::CONFIRMATION: return "CONFIRMATION";
    case gkv::ResponsePacketType::DEVICE_INFO: return "DEVICE_INFO";
    case gkv::ResponsePacketType::DEVICE_SETTINGS: return "DEVICE_SETTINGS";
    case gkv::ResponsePacketType::CUSTOM_PACKET_SETTINGS: return "CUSTOM_PACKET_SETTINGS";
    case gkv::ResponsePacketType::DATA_ADC_CODES: return "DATA_ADC_CODES";
    case gkv::ResponsePacketType::DATA_CALIBRATED_DATA: return "DATA_CALIBRATED_DATA";
    case gkv::ResponsePacketType::DATA_ORIENTATION: return "DATA_ORIENTATION";
    case gkv::ResponsePacketType::DATA_INCLINOMETER: return "DATA_INCLINOMETER";
    case gkv::ResponsePacketType::DATA_BINS: return "DATA_BINS";
    case gkv::ResponsePacketType::DATA_CUSTOM_PACKET: return "DATA_CUSTOM_PACKET";
    case gkv::ResponsePacketType::DATA_GNSS: return "DATA_GNSS";
    case gkv::ResponsePacketType::DATA_GNSS_EXTENDED: return "DATA_GNSS_EXTENDED";
    case gkv::ResponsePacketType::ANGULAR_VELOCITY_OFFSET_COEFS: return "ANGULAR_VELOCITY_OFFSET_COEFS";
    case gkv::ResponsePacketType::GYRO_OFFSET_COEFS: return "GYRO_OFFSET_COEFS";
    case gkv::ResponsePacketType::FILTER_SETTINGS: return "FILTER_SETTINGS";
    case gkv::ResponsePacketType::ALGORITHM_SETTINGS: return "ALGORITHM_SETTINGS";
    default: return "unknown packet type";
  }
}

} // namespace

class GkvParser : public Parser {
 public:
  GkvParser();
  explicit GkvParser(const config::Config& config);
  MessageType GetMessage(MessagePtr* message_ptr) override;

 private:
  Parser::MessageType PrepareMessage(MessagePtr* message_ptr);
  bool HandleCustomPacket(const gkv::CustomPacket* packet);

 private:
//   size_t total_length_ = 0;
  std::vector<uint8_t> buffer_;
};

Parser* Parser::CreateGkv(const config::Config& config) {
  return new GkvParser(config);
}

GkvParser::GkvParser() {
  buffer_.reserve(BUFFER_SIZE);
//   ins_.mutable_position_covariance()->Resize(9, FLOAT_NAN);
//   ins_.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
//   ins_.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);

//   if (1 != init_raw(&raw_)) {
//     AFATAL << "memory allocation error for observation data structure.";
//   }
}

GkvParser::GkvParser(const config::Config& config) {
  buffer_.reserve(BUFFER_SIZE);
//   ins_.mutable_position_covariance()->Resize(9, FLOAT_NAN);
//   ins_.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
//   ins_.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);

//   if (config.has_imu_type()) {
//     imu_type_ = config.imu_type();
//   }

//   if (1 != init_raw(&raw_)) {
//     AFATAL << "memory allocation error for observation data structure.";
//   }
}

Parser::MessageType GkvParser::GetMessage(MessagePtr* message_ptr) {
  if (data_ == nullptr) {
    return MessageType::NONE;
  }

  while (data_ < data_end_) {
    // looking for preamble
    if (buffer_.empty()) {
      if (*data_ == gkv::PACKET_PREAMBLE) {
        buffer_.push_back(*data_);
      }
      ++data_;
      continue;
    }
    assert(buffer_[0] == gkv::PACKET_PREAMBLE);

    // gather header
    if (!buffer_.empty() && buffer_.size() < sizeof(gkv::Header)) {
      buffer_.push_back(*data_);
      ++data_;
      continue;
    }

    // read data length
    assert(buffer_.size() >= sizeof(gkv::Header));
    const auto header = reinterpret_cast<gkv::Header*>(buffer_.data());
    const size_t total_length = sizeof(gkv::Header) + header->data_length + gkv::CRC_LENGTH;

    // gather data and CRC
    if (buffer_.size() < total_length) {
      buffer_.push_back(*data_);
      ++data_;
    }
    else {
      // process packet
      const size_t l = buffer_.size() - gkv::CRC_LENGTH;
      const uint32_t crc_received = *reinterpret_cast<uint32_t*>(buffer_.data() + l);
      const uint32_t crc_calculated = crc32(0, buffer_.data(), l);

      if (crc_calculated == crc_received) {
        // valid packet
        MessageType type = PrepareMessage(message_ptr);
        buffer_.clear();
        if (type != MessageType::NONE) {
            return type;
        }
      }
      else {
        AWARN << "False 0xFF start flag detected or bad CRC!";
        // false preamble detected or invalid packet CRC
        const auto iter = std::find(buffer_.begin() + 1, buffer_.end(), gkv::PACKET_PREAMBLE);
        buffer_.erase(buffer_.begin(), iter);
      }
    }
  }

  return MessageType::NONE;
}

Parser::MessageType GkvParser::PrepareMessage(MessagePtr* message_ptr) {
  if (!message_ptr) {
    return MessageType::NONE;
  }

  uint8_t* data = buffer_.data() + sizeof(gkv::Header);
  const auto header = reinterpret_cast<const gkv::Header*>(buffer_.data());

  // AINFO << "----- Got new message -----";
  // AINFO << "Packet type: " << ResponsePacketTypeToString((gkv::ResponsePacketType)header->packet_type);
  // AINFO << "Address: " << (size_t)header->address;
  // AINFO << "Data length: " << (size_t)header->data_length;

  switch(header->packet_type) {
    case gkv::ResponsePacketType::DATA_CUSTOM_PACKET:
      if (header->data_length != sizeof(gkv::CustomPacket)) {
        AERROR << "Incorrect data_length for packet with type: " <<
                  ResponsePacketTypeToString(gkv::ResponsePacketType::DATA_CUSTOM_PACKET);
        break;
      }
      if (HandleCustomPacket(reinterpret_cast<gkv::CustomPacket*>(data))) {
        // TODO: create own proto message
        // *message_ptr = &gnss_;
        // return MessageType::GKV_CUSTOM_PACKET;
        return MessageType::NONE; // TODO: stub for testing
      }
      break;

    // case gkv::ResponsePacketType::DATA_GNSS:
    //   if (header->data_length != sizeof(gkv::GNSS)) {
    //     AERROR << "Incorrect data_length for packet with type: " <<
    //               ResponsePacketTypeToString(gkv::ResponsePacketType::DATA_CUSTOM_PACKET);
    //     break;       
    //   }
    //   /// TODO: test only
    //   AINFO << "GNSS data [lat]: " << reinterpret_cast<gkv::GNSS*>(data)->latitude;
    //   AINFO << "GNSS data [lon]: " << reinterpret_cast<gkv::GNSS*>(data)->longitude;
    //   break;

    default:
      break;
  }

  return MessageType::NONE;
}

bool GkvParser::HandleCustomPacket(const gkv::CustomPacket* packet) {
  // TODO: implement
  // TODO: fill proto message
  return false;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo