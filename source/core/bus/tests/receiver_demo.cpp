#include "source/core/bus/bus.h"

#include <iostream>
#include <thread>

int main() {
    mte::bus::Receiver<std::string> str_receiver("test_channel_1");

    while (true) {
        const auto msg = str_receiver.ReceiveLatest();

        if (msg.has_value()) {
            std::cout << "Message received: " << msg.value() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds{2});
        } else {
            std::cout << "No message received" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds{500});
        }
    }
}
