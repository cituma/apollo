load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libapollo_to_ros.so",
    linkshared = True,
    linkstatic = False,
    deps = [":apollo_to_ros_lib"],
)

cc_library(
    name = "apollo_to_ros_lib",
    srcs = ["apollo_to_ros.cc"],
    hdrs = ["apollo_to_ros.h"],
    deps = [
        "//cyber",
        "//modules/drivers/proto:pointcloud_cc_proto",
    ],
)

cpplint()
