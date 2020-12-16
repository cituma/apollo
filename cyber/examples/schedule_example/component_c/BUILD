load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libcomponent_c.so",
    linkshared = True,
    linkstatic = False,
    deps = [":component_c_lib"],
)

cc_library(
    name = "component_c_lib",
    srcs = ["component_c.cc"],
    hdrs = ["component_c.h"],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)

cpplint()
