#include "source/core/bus/bus.h"

#include "source/localization/pose.h"

#include <iostream>
#include <thread>

int main() {
    mte::bus::Sender<mte::localization::Pose> pose_sender{"test_channel_1"};

    const mte::geometry::Vector3d pos{1, 2, 3};
    const mte::geometry::Vector3d vel{4, 5, 6};
    const mte::geometry::Vector3d ori{7, 8, 9};
    const mte::geometry::Vector3d ang{10, 11, 12};

    mte::localization::Pose p{pos, vel, ori, ang};

    for (int i = 0; i != 100; ++i) {
        p.x += 1;
        std::cout << p.Position().transpose() << std::endl;
        pose_sender.Send(p);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
