load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

TELEMETRY_COPTS = ['-DMODULE_NAME=\\"telemetry\\"']

cc_binary(
    name = "libtelemetry_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":telemetry_component_lib"],
)

cc_library(
    name = "telemetry_component_lib",
    srcs = ["telemetry_component.cc",
            "output.cc"],
    hdrs = ["telemetry_component.h",
            "output.h"],
    copts = TELEMETRY_COPTS,
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common/monitor_log",
        "//modules/localization/msf/common/util:frame_transform",
        "//modules/common/math:math",
        "//modules/telemetry/proto:config_cc_proto",
        "//modules/telemetry/proto:telemetry_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/basic_msgs:vehicle_signal_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/map/hdmap:hdmap_util",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/planning_msgs:pad_msg_cc_proto",
    ],
)

install(
    name = "install",
    library_dest = "telemetry/lib",
    data_dest = "telemetry",
    data = [
        ":runtime_data",
        # ":cyberfile.xml",
        "telemetry.BUILD",
    ],
    targets = [
        ":libtelemetry_component.so",
    ],
    deps = [
        ":pb_hdrs",
        "//modules/telemetry/proto:py_pb_telemetry",
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "telemetry/include",
    data = [
        "//modules/telemetry/proto:telemetry_cc_proto",
        "//modules/telemetry/proto:config_cc_proto",      
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_telemetry_src",
        ":install_telemetry_hdrs"
    ],
)

install_src_files(
    name = "install_telemetry_src",
    src_dir = ["."],
    dest = "telemetry/src",
    filter = "*",
)

install_src_files(
    name = "install_telemetry_hdrs",
    src_dir = ["."],
    dest = "telemetry/include",
    filter = "*.h",
)

filegroup(
    name = "runtime_data",
    srcs = glob([
       "conf/*.txt",
       "dag/*.dag",
       "launch/*.launch",
    ]),
)

cpplint()
