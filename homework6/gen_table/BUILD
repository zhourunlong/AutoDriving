package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "gen_table",
    srcs = ["gen_table.cc"],
    deps = [
        "//homework6/agents/vz:vz_agent",
        "//homework6/simulation:vehicle_agent_factory",
        "//homework6/simulation:simulation_world",
        "//homework6/simulation:simulation_system",
        "//homework6/simulation/dynamic_lib:libdynamic",
        "@gflags",
        "@glog",
    ],
)
