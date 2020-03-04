#pragma once

#include "msgpack.hpp"

#include <sstream>
#include <type_traits>

namespace mte {
namespace bus {

// NOTE(jacob): The `concept bool` syntax is from the Concepts TS, not the final feature that made
// it into C++20. Unfortunately, we're stuck on GCC 8 for now, which doesn't have full support for
// that. If we ever wanted to modernize this codebase, this would be one of the things to change.

// NOTE: clang-format isn't very good at autoformatting concepts, so we disable it here.
// clang-format off
template <typename T>
concept bool CustomSerializable =
    requires(T x, std::string s) {
        x.Serialize();
        { T::Deserialize(s) } -> T;
    };

// NOTE(jacob): For some reason, requiring msgpack::pack and obj.convert isn't sufficient to filter
// out types that will actually fail to compile with msgpack serialization, so we need to explicitly
// require that those types are not CustomSerializable.
template <typename T>
concept bool MsgpackSerializable =
    !CustomSerializable<T> &&
    std::is_default_constructible<T>::value &&
    requires(T x, std::stringstream ss, msgpack::object obj) {
        msgpack::pack(ss, x);
        obj.convert(x);
    };
// clang-format on

// The MsgpackSerializable serialization and deserialization functions will be used for
// serialization of simple types that can rely on msgpack's built-in serialization. This includes
// classes where the only data members are primitive types, vectors, or other simple type classes.
// See MsgpackSerializableClass in serialization_tests.cpp for an example.

template <MsgpackSerializable T>
std::string Serialize(const T& gen_obj) {
    std::stringstream ss;
    msgpack::pack(ss, gen_obj);
    return ss.str();
}

template <MsgpackSerializable T>
T Deserialize(const std::string& buf) {
    T gen_obj;
    msgpack::object_handle oh = msgpack::unpack(buf.data(), buf.size());
    msgpack::object obj = oh.get();
    return obj.convert(gen_obj);
}

// The CustomSerializable serialization and deserialization functions will be used for serialization
// of more complicated types for which custom serialization is necessary or desired (such as Eigen
// types). See CustomSerializableClass in serialization_tests.cpp for an example.

template <CustomSerializable T>
std::string Serialize(const T& gen_obj) {
    return gen_obj.Serialize();
}

template <CustomSerializable T>
T Deserialize(const std::string& buf) {
    return T::Deserialize(buf);
}

}  // namespace bus
}  // namespace mte
