load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcomponent_input.so",
    linkshared = True,
    linkstatic = False,
    deps = [":component_input_lib"],
)

cc_library(
    name = "component_input_lib",
    srcs = ["component_input.cc"],
    hdrs = ["component_input.h"],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)

cpplint()
