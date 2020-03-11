#include "icp.h"
#include "localizer.h"
#include "pose.h"
#include "source/core/bus/bus.h"
#include "source/core/calibration.h"
#include "source/core/logging.h"
#include "source/sensing/lidar/lidar_types.h"

#include <chrono>
#include <thread>

using namespace std::chrono_literals;

constexpr auto kLoopDuration{500ms};

// TODO(jacob): Set this based on experimental testing.
constexpr double kInlierDist{-1};

using namespace mte;

int main() {
    InitializeLogging("localizer");

    bus::Sender<localization::Pose> sender("pose");
    bus::Receiver<lidar::PointCloud> lidar_receiver("lidar_points");

    auto last_pose = calibration::initial_pose;

    while (true) {
        const auto loop_start = std::chrono::system_clock::now();

        const auto points = lidar_receiver.ReceiveLatest();
        const auto model_points = localization::GetVisibleWallPoints(last_pose);

        if (points.has_value()) {
            const auto lidar_pose = localization::LocalizeToPointCloud(model_points, points.value(),
                                                                       last_pose, kInlierDist);
            last_pose = lidar_pose;
            sender.Send(last_pose);
        } else {
            LOG_WARN("No point cloud received");
        }

        std::this_thread::sleep_until(loop_start + kLoopDuration);
    }
}
