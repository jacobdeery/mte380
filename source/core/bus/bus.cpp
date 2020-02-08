#include "bus.h"

#include "serialization.h"

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
    {"plan",                  "13003"}
    // clang-format on
};

std::string GetChannelEndpoint(const std::string& channel_name) {
    return transport_address + ":" + channel_ports.at(channel_name);
}

}  // namespace bus
}  // namespace mte
