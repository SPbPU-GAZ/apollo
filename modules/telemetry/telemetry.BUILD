load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "telemetry",
    includes = ["include"],
    hdrs = glob(["include/**/*.h"]),
    srcs = glob(["lib/**/lib*.so*"]),
    include_prefix = "modules/telemetry",
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)
