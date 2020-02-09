#include "source/core/bus/bus.h"

#include "source/localization/pose.h"

#include <iostream>
#include <thread>

int main() {
    mte::bus::Receiver<mte::localization::Pose> pose_receiver("test_channel_1");

    while (true) {
        const auto msg = pose_receiver.ReceiveLatest();

        if (msg.has_value()) {
            std::cout << "Message received: " << msg.value().Position().transpose() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds{2});
        } else {
            std::cout << "No message received" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds{500});
        }
    }
}
