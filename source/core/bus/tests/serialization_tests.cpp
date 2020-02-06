#include <gtest/gtest.h>

#include "source/core/bus/serialization.h"

using namespace mte::bus;

class SerializableClass {
   public:
    SerializableClass() = default;
    SerializableClass(int x, const std::string& s) : x{x}, s{s} {}

    int X() const { return x; };
    std::string S() const { return s; };

   private:
    int x;
    std::string s;

   public:
    MSGPACK_DEFINE(x, s);
};

TEST(SerializationTests, IntRoundTrip) {
    const int i = 5;
    const auto buf = Serialize(i);
    const auto deserialized = Deserialize<int>(buf);

    EXPECT_EQ(i, deserialized);
}

TEST(SerializationTests, CustomClassRoundTrip) {
    const SerializableClass stc(10, "mte");
    const auto buf = Serialize(stc);
    const auto deserialized = Deserialize<SerializableClass>(buf);

    EXPECT_EQ(stc.X(), deserialized.X());
    EXPECT_EQ(stc.S(), deserialized.S());
}
