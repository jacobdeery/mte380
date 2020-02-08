#pragma once

#include "serialization.h"

#include "zmqpp.hpp"

#include <optional>
#include <string>

namespace mte {
namespace bus {

std::string GetChannelEndpoint(const std::string& channel_name);

template <class T>
class Sender {
   public:
    Sender(const std::string& channel_name)
        : ctx{zmqpp::context()}, sock{zmqpp::socket(ctx, zmqpp::socket_type::publish)} {
        sock.bind(GetChannelEndpoint(channel_name));
    }

    void Send(const T& obj) {
        const auto buf = Serialize(obj);
        zmqpp::message message;
        message << buf;
        sock.send(message);
    }

   private:
    zmqpp::context ctx;
    zmqpp::socket sock;
};

template <class T>
class Receiver {
   public:
    Receiver(const std::string& channel_name)
        : ctx{zmqpp::context()}, sock{zmqpp::socket(ctx, zmqpp::socket_type::subscribe)} {
        sock.connect(GetChannelEndpoint(channel_name));
        sock.subscribe("");
    }

    std::optional<T> ReceiveLatest() {
        zmqpp::message message;
        zmqpp::message tmp;
        while (sock.receive(tmp, /* dont_block = */ true)) {
            message = std::move(tmp);
        }
        if (message.parts() != 0) {
            std::string buf;
            message >> buf;
            return Deserialize<T>(buf);
        } else {
            return std::nullopt;
        }
    }

    std::vector<T> ReceiveMultiple(size_t max) {
        std::vector<T> messages;
        messages.reserve(max);
        zmqpp::message message;
        while (sock.receive(message, /* dont_block = */ true) && messages.size() < max) {
            std::string buf;
            message >> buf;
            messages.emplace_back(Deserialize<T>(buf));
        }
        return messages;
    }

   private:
    zmqpp::context ctx;
    zmqpp::socket sock;
};

}  // namespace bus
}  // namespace mte
