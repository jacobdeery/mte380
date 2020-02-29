#include "source/core/bus/bus.h"
#include "source/core/logging.h"
#include "ydlidar_bridge.h"

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

constexpr auto kLoopDuration{500ms};

using namespace mte;

int main() {
    InitializeLogging("lidar_driver");

    lidar::LidarBridge lidar_bridge;

    // TODO(jacob): Remove after the demo
    bus::Sender<char> planner_command_sender("planner_command");

    while (!lidar_bridge.Initialize()) {
        LOG_INFO("Unable to initialize the lidar bridge. Retrying in 1 second...");
        std::this_thread::sleep_for(1s);
    }

    while (true) {
        const auto loop_start = std::chrono::system_clock::now();

        const auto scan = lidar_bridge.Scan();
        if (scan.has_value()) {
            if (lidar::IsThereAWall(scan.value())) {
                planner_command_sender.Send('1');
            }
        } else {
            LOG_WARN("Lidar scan had no value");
        }

        std::this_thread::sleep_until(loop_start + kLoopDuration);
    }
}
