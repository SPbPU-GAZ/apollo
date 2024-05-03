load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "gazelle_vehicle_factory",
    srcs = ["gazelle_vehicle_factory.cc"],
    hdrs = ["gazelle_vehicle_factory.h", "http.h"],
    copts = CANBUS_COPTS,
    alwayslink = True,
    deps = [
        "//modules/common/status",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_binary(
    name = "libgazelle_vehicle_factory_lib.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":gazelle_vehicle_factory"
    ],
)

install(
    name = "install",
    library_dest = "canbus-vehicle-gazelle/lib",
    data_dest = "canbus-vehicle-gazelle",
    data = [
        # ":runtime_data",
        # ":cyberfile.xml", TODO: what is it?
        ":canbus-vehicle-gazelle.BUILD",
    ],
    targets = [
        ":libgazelle_vehicle_factory_lib.so",
    ],
    # deps = [
    #     ":pb_lincoln",
    #     ":pb_hdrs",
    # ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_gazelle_src",
        ":install_gazelle_hdrs"
    ],
)

install_src_files(
    name = "install_gazelle_src",
    src_dir = ["."],
    dest = "canbus-vehicle-gazelle/src",
    filter = "*",
)

install_src_files(
    name = "install_gazelle_hdrs",
    src_dir = ["."],
    dest = "canbus-vehicle-gazelle/include",
    filter = "*.h",
)

cpplint()