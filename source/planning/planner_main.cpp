#include "arduino_bridge.h"
#include "planner.h"
#include "source/core/bus/bus.h"
#include "source/core/logging.h"
#include "source/localization/pose.h"

#include <thread>

using namespace mte;
using namespace std::chrono_literals;

constexpr auto kLoopDuration{100ms};

// TODO(jacob): This file is pretty messy. Returning optional strings is not a clean design.
// Figure out a better way to handle bridge errors.

int main() {
    InitializeLogging("planner");

    ArduinoBridge bridge;

    auto connection_error_message = bridge.Connect();
    while (connection_error_message.has_value()) {
        LOG_INFO("Failed to connect to Arduino with error message: " +
                 connection_error_message.value() + ". Retrying in 3 seconds.");
        std::this_thread::sleep_for(2s);
        connection_error_message = bridge.Connect();
    }

    LOG_INFO("Connected to Arduino.");

    const auto write_error_message = bridge.Send(planning::MakeConstantSpeedPlan(60, 60));
    if (write_error_message.has_value()) {
        LOG_WARN("Arduino bridge failed to send data with error message: " +
                 write_error_message.value());
    }

    while (true) {
        const auto loop_start = std::chrono::system_clock::now();
        std::this_thread::sleep_until(loop_start + kLoopDuration);
    }
}
