load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gnss_util",
    srcs = [
        "gnss_conversion.cc",
    ],
    hdrs = [
        "macros.h",
        "time_conversion.h",
        "gnss_conversion.h",
    ],
)

cpplint()
