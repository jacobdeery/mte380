#include "bus.h"

#include "serialization.h"
#include "source/core/logging.h"

namespace mte {
namespace bus {

const std::string transport_address{"tcp://127.0.0.13"};

std::unordered_map<std::string, std::string> channel_ports = {
    // clang-format off
    {"test_channel_1",        "12000"},
    {"test_channel_2",        "12001"},

    {"lidar_points",          "13000"},
    {"imu_data",              "13001"},
    {"pose",                  "13002"},
    {"plan",                  "13003"},

    {"planner_command",       "14000"}
    // clang-format on
};

std::string GetChannelEndpoint(const std::string& channel_name) {
    CHECK(channel_ports.count(channel_name) == 1,
          "Unknown channel specified. Add the channel name to bus.cpp.");
    return transport_address + ":" + channel_ports.at(channel_name);
}

}  // namespace bus
}  // namespace mte
