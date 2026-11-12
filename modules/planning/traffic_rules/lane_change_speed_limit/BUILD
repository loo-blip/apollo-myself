load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

filegroup(
    name = "runtime_files",
    srcs = glob(["conf/**"]),
)

apollo_plugin(
    name = "liblane_change_speed_limit.so",
    description = ":plugins.xml",
    srcs = ["lane_change_speed_limit.cc"],
    hdrs = ["lane_change_speed_limit.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/traffic_rules/lane_change_speed_limit/proto:lane_change_speed_limit_cc_proto",
    ],
)

apollo_package()

cpplint()
