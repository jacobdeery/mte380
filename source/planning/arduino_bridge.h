#pragma once

#include <termios.h>

#include <optional>

namespace mte {

class ArduinoBridge {
   public:
    ArduinoBridge();
    ~ArduinoBridge();

    // These functions will return a std::nullopt if no error occurred.
    std::optional<std::string> Connect();
    std::optional<std::string> Send(char c) const;

   private:
    ArduinoBridge(const ArduinoBridge&) = delete;
    ArduinoBridge& operator=(const ArduinoBridge&) = delete;

    std::optional<int> serial_port;
    struct termios tty;
};

}  // namespace mte
