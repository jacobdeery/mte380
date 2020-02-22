#pragma once

#include "msgpack.hpp"

#include <sstream>

namespace mte {
namespace bus {

template <class T>
std::string Serialize(const T& gen_obj) {
    std::stringstream ss;
    msgpack::pack(ss, gen_obj);
    return ss.str();
}

template <class T>
T Deserialize(const std::string& buf) {
    T gen_obj;
    msgpack::object_handle oh = msgpack::unpack(buf.data(), buf.size());
    msgpack::object obj = oh.get();
    return obj.convert(gen_obj);
}

}  // namespace bus
}  // namespace mte
