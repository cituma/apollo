load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcomponent_a.so",
    linkshared = True,
    linkstatic = False,
    deps = [":component_a_lib"],
)

cc_library(
    name = "component_a_lib",
    srcs = ["component_a.cc"],
    hdrs = ["component_a.h"],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)

cpplint()
