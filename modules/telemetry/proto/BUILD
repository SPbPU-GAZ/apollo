load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")
load("//tools/install:install.bzl", "install", "install_files")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "config_cc_proto",
    deps = [
        ":config_proto",
    ],
)

proto_library(
    name = "config_proto",
    srcs = ["config.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
        "//modules/common_msgs/basic_msgs:geometry_proto",
    ],
)

py_proto_library(
    name = "config_py_pb2",
    deps = [
        ":config_proto",
    ],
)

cc_proto_library(
    name = "telemetry_cc_proto",
    deps = [
        ":telemetry_proto",
    ],
)

proto_library(
    name = "telemetry_proto",
    srcs = ["telemetry.proto"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_proto",
    ],
)

py_proto_library(
    name = "telemetry_py_pb2",
    deps = [
        ":telemetry_proto",
    ],
)

install_files(
    name = "py_pb_telemetry",
    dest = "telemetry/python/modules/telemetry/proto",
    files = [
        "config_py_pb2",
        "telemetry_py_pb2",
    ]
)
