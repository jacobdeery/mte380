#include <gtest/gtest.h>

#include "source/core/bus/serialization.h"

using namespace mte;

class MsgpackSerializableClass {
   public:
    MsgpackSerializableClass() = default;
    MsgpackSerializableClass(int x, const std::string& s) : x{x}, s{s} {}

    int X() const { return x; };
    std::string S() const { return s; };

   private:
    int x;
    std::string s;

   public:
    MSGPACK_DEFINE(x, s);
};

class CustomSerializableClass {
   public:
    CustomSerializableClass(int x, int y, int z) : x{x}, y{y}, z{z} {}

    std::tuple<int, int, int> XYZ() const { return {x, y, z}; };

   private:
    int x;
    int y;
    int z;

   public:
    std::string Serialize() const {
        return std::to_string(x) + "/" + std::to_string(y) + "/" + std::to_string(z);
    }

    static CustomSerializableClass Deserialize(const std::string& buf) {
        const size_t ypos = buf.find("/");
        const int x_ds = std::stoi(buf.substr(0, ypos));
        const size_t zpos = buf.find("/", ypos + 1);
        const int y_ds = std::stoi(buf.substr(ypos + 1, zpos - ypos));
        const int z_ds = std::stoi(buf.substr(zpos + 1, buf.size() - zpos));
        return CustomSerializableClass(x_ds, y_ds, z_ds);
    }
};

TEST(SerializationTests, IntRoundTrip) {
    const int i = 5;
    const auto buf = bus::Serialize(i);
    const auto deserialized = bus::Deserialize<int>(buf);

    EXPECT_EQ(i, deserialized);
}

TEST(SerializationTests, MsgpackClassRoundTrip) {
    const MsgpackSerializableClass msc(10, "mte");
    const auto buf = bus::Serialize(msc);
    const auto deserialized = bus::Deserialize<MsgpackSerializableClass>(buf);

    EXPECT_EQ(msc.X(), deserialized.X());
    EXPECT_EQ(msc.S(), deserialized.S());
}

TEST(SerializationTests, CustomClassRoundTrip) {
    const int x = 100;
    const int y = 2;
    const int z = 30;

    const CustomSerializableClass csc(x, y, z);
    const auto buf = bus::Serialize(csc);
    const auto deserialized = bus::Deserialize<CustomSerializableClass>(buf);

    const auto [x2, y2, z2] = deserialized.XYZ();

    EXPECT_EQ(x, x2);
    EXPECT_EQ(y, y2);
    EXPECT_EQ(z, z2);
}
