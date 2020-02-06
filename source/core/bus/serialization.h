#pragma once

#include <msgpack.hpp>

namespace mte {
namespace bus {

template <class T>
msgpack::sbuffer Serialize(const T& gen_obj) {
    msgpack::sbuffer buf;
    msgpack::pack(buf, gen_obj);
    return buf;
}

template <class T>
T Deserialize(const msgpack::sbuffer& buf) {
    T gen_obj;
    msgpack::object_handle oh = msgpack::unpack(buf.data(), buf.size());
    msgpack::object obj = oh.get();
    return obj.convert(gen_obj);
}

}  // namespace bus
}  // namespace mte
