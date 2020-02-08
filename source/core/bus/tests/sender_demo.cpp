#include "source/core/bus/bus.h"

#include <iostream>
#include <thread>

int main() {
    mte::bus::Sender<std::string> str_sender{"test_channel_1"};

    for (int i = 0; i != 100; ++i) {
        std::string str{"MTE 380 " + std::to_string(i)};
        std::cout << str << std::endl;
        str_sender.Send(str);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
