load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "control_panel_stream",
    deps = [
        ":serial_stream",
    ],
)

cc_library(
    name = "serial_stream",
    srcs = ["serial_stream.cc"],
    deps = [
        ":stream",
        "//cyber",
    ],
)

cc_library(
    name = "stream",
    hdrs = ["stream.h", "macros.h"],
    deps = [
        "//cyber",
    ],
)

cpplint()
