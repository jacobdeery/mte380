#include "lidar_types.h"
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

    bus::Sender<lidar::PointCloud> sender("lidar_points");

    while (!lidar_bridge.Initialize()) {
        LOG_INFO("Unable to initialize the lidar bridge. Retrying in 1 second...");
        std::this_thread::sleep_for(1s);
    }

    while (true) {
        const auto loop_start = std::chrono::system_clock::now();

        const auto scan = lidar_bridge.Scan();
        if (scan.has_value()) {
            const auto pc = lidar::PointCloud(scan.value());
            sender.Send(pc);
        } else {
            LOG_WARN("Lidar scan had no value");
        }

        std::this_thread::sleep_until(loop_start + kLoopDuration);
    }
}
