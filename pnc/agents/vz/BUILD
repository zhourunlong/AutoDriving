package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vz_agent",
    hdrs = ["vz_agent.h"],
    deps = [
        "//common/proto:cc_agent_status_proto",
        "//common/proto:cc_control_proto",
        "//common/proto:cc_simulation_proto",
        "//common/proto:cc_route_proto",
        "//pnc/simulation:vehicle_agent",
        "//pnc/simulation:vehicle_agent_factory",
        "//common/utils/file",
        "@glog",
    ],
)
