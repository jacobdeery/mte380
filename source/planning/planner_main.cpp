#include "arduino_bridge.h"
#include "planner.h"
#include "source/core/bus/bus.h"
#include "source/core/logging.h"
#include "source/localization/pose.h"

#include <thread>

using namespace mte;
using namespace std::chrono_literals;

constexpr auto kLoopDuration{100ms};

int main() {
    InitializeLogging("planner");

    ArduinoBridge bridge;

    bool connection_success = bridge.Connect();
    while (!connection_success) {
        LOG_WARN("Failed to connect to Arduino. Retrying in 2 seconds.");
        std::this_thread::sleep_for(2s);
        connection_success = bridge.Connect();
    }

    LOG_INFO("Connected to Arduino.");

    if (!bridge.Send(planning::MakeConstantSpeedPlan(60, 60))) {
        LOG_WARN("Arduino bridge failed to send data.");
    }

    while (true) {
        const auto loop_start = std::chrono::system_clock::now();
        std::this_thread::sleep_until(loop_start + kLoopDuration);
    }
}
