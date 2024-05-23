load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

LS180S2_COPTS = ['-DMODULE_NAME=\\"ls180s2\\"']

cc_library(
    name = "driver",
    srcs = ["driver.cc"],
    hdrs = ["driver.h", "ThreadPool.h"],
    copts = LS180S2_COPTS,
    linkopts = [
        "-lpthread",
    ],
    deps = [
        "//cyber",
        "//modules/drivers/lidar/proto:ls180s2_cc_proto",
        "//modules/drivers/lidar/proto:ls180s2_config_cc_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/common/driver_factory:driver_base",
        "@local_config_pcl//:pcl",
        ":input",
        ":parsing",
    ],
)

cc_library(
    name = "input",
    srcs = ["input.cc"],
    hdrs = ["input.h"],
    deps = [
        "//cyber",
        "//modules/drivers/lidar/proto:ls180s2_cc_proto",
    ],
)

cc_library(
    name = "parsing",
    srcs = ["parsing.cc"],
    hdrs = ["parsing.h"],
    deps = [
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
    ],
)

cpplint()
