load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcomponent_b.so",
    linkshared = True,
    linkstatic = False,
    deps = [":component_b_lib"],
)

cc_library(
    name = "component_b_lib",
    srcs = ["component_b.cc"],
    hdrs = ["component_b.h"],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)

cpplint()
