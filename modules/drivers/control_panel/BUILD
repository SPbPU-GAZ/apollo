load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CONTROL_PANEL_COPTS = ['-DMODULE_NAME=\\"control_panel\\"']

cc_binary(
    name = "libcontrol_panel_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":control_panel_component_lib"],
)

cc_library(
    name = "control_panel_component_lib",
    srcs = ["control_panel_component.cc"],
    hdrs = ["control_panel_component.h"],
    copts = CONTROL_PANEL_COPTS,
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common/monitor_log",
        "//modules/drivers/control_panel/proto:config_cc_proto",
        "//modules/common_msgs/planning_msgs:pad_msg_cc_proto",
        "//modules/drivers/control_panel/stream:control_panel_stream"
    ],
)

install(
    name = "install",
    data_dest = "drivers/addition_data/control_panel",
    library_dest = "drivers/lib/control_panel",
    data = [
        ":runtime_data",
    ],
    targets = [
        ":libcontrol_panel_component.so",
    ],
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
