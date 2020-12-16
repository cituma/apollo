load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcomponent_d.so",
    linkshared = True,
    linkstatic = False,
    deps = [":component_d_lib"],
)

cc_library(
    name = "component_d_lib",
    srcs = ["component_d.cc"],
    hdrs = ["component_d.h"],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)

cpplint()
