#include "arduino_bridge.h"

#include "source/core/logging.h"

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include <optional>
#include <string>

namespace mte {

// TODO(jacob): Figure out a robust way of determining at runtime what port the Arduino is on.
const std::string port_name{"/dev/ttyUSB1"};

// TODO(jacob/taylor): Tune this based on planner/controller integration testing.
constexpr auto baud{B9600};

ArduinoBridge::ArduinoBridge() {
    memset(&tty, 0, sizeof(tty));
}

std::optional<std::string> ArduinoBridge::Connect() {
    // Adapted from:
    // blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp

    if (!serial_port.has_value()) {
        serial_port = open(port_name.c_str(), O_WRONLY);
        if (serial_port < 0) {
            serial_port = std::nullopt;
            return strerror(errno);
        }
    }

    if (tcgetattr(serial_port.value(), &tty) != 0) {
        return strerror(errno);
    }

    tty.c_cflag |= CS8;      // 8 data bits
    tty.c_cflag &= ~CSTOPB;  // 1 stop bit
    tty.c_cflag &= ~PARENB;  // No parity bit

    tty.c_cflag &= ~CRTSCTS;  // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CLOCAL;    // Ignore control lines

    tty.c_oflag &= ~OPOST;  // Prevent special interpretation of output bytes
    tty.c_oflag &= ~ONLCR;  // Prevent conversion of newline to carriage return/line feed

    cfsetospeed(&tty, baud);

    if (tcsetattr(serial_port.value(), TCSANOW, &tty) != 0) {
        return strerror(errno);
    }

    return std::nullopt;
}

std::optional<std::string> ArduinoBridge::Send(char c) const {
    if (!serial_port.has_value()) {
        return "Arduino bridge is not connected";
    } else if (write(serial_port.value(), &c, sizeof(c)) < 0) {
        return strerror(errno);
    } else {
        return std::nullopt;
    }
}

ArduinoBridge::~ArduinoBridge() {
    if (serial_port.has_value()) {
        close(serial_port.value());
    }
}

}  // namespace mte
