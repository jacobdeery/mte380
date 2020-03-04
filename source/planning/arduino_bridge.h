#pragma once

#include "planner.h"

#include <termios.h>

#include <optional>

namespace mte {

class ArduinoBridge {
   public:
    ArduinoBridge(){};
    ~ArduinoBridge();

    // These functions will return a std::nullopt if no error occurred.
    std::optional<std::string> Connect();
    std::optional<std::string> Send(const planning::WheelSpeedPlan& plan) const;

   private:
    ArduinoBridge(const ArduinoBridge&) = delete;
    ArduinoBridge& operator=(const ArduinoBridge&) = delete;

    std::optional<int> serial_port;
};

constexpr int kBytesInSerializedPlan{2 * planning::kNumWheelSpeedPoints * sizeof(float)};

struct SerializedPlan {
    SerializedPlan(const planning::WheelSpeedPlan& plan);
    char data[kBytesInSerializedPlan];
};

}  // namespace mte
