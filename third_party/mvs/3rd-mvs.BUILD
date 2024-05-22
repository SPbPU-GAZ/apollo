load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

cc_library(
    name = "hik",
    includes = ["include"],
    srcs = select(
        {
            "@platforms//cpu:x86_64": glob(["lib/64/*.so"]),
            "@platforms//cpu:aarch64": glob(["lib/aarch64/*.so"]),
        },
        no_match_error = "Please Build with an ARM or Linux x86_64 platform",
    ),
    visibility = ["//visibility:public"],
)
