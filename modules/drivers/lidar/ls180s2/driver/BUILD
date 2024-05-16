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
        "//modules/drivers/lidar/common/driver_factory:driver_base",
        "@local_config_pcl//:pcl",
        # "@eigen",
        ":input",
    ],
)

cc_library(
    name = "input",
    srcs = ["input.cc"],
    hdrs = ["input.h"],
    copts = LS180S2_COPTS,
    deps = [
        "//cyber",
        "//modules/drivers/lidar/proto:ls180s2_cc_proto",
    ],
)

cpplint()
