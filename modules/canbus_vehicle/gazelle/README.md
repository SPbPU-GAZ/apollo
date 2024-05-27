//   static constexpr int64_t http_timeout_ms = 10000;
// //   static const constexpr char* get_url = "http://httpbin.org/anything"; // "http://ip.ip.ip.ip:port/api/chassis_state";
// //   static const constexpr char* post_url = "http://httpbin.org/post";    // "http://ip.ip.ip.ip:port/api/control_command";
//   static const constexpr char* get_url = "http://192.168.100.100:5000/api/chassis_state";
//   static const constexpr char* post_url = "http://192.168.100.100:5000/api/control_command";

export GLOG_alsologtostderr=1

cyber/tools/cyber_launch/cyber_launch.py start /apollo/modules/canbus/launch/canbus.launch