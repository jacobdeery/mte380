#include "source/core/bus/bus.h"

#include "gtest/gtest.h"

#include <thread>

using namespace mte::bus;

TEST(BusTests, GetChannelEndpointTest) {
    EXPECT_EQ("tcp://127.0.0.13:12000", GetChannelEndpoint("test_channel_1"));
}

TEST(BusTests, UnknownChannelThrows) {
    EXPECT_ANY_THROW(GetChannelEndpoint("unknown_channel"));
}

TEST(BusTests, SendAndReceiveSingleMessage) {
    const std::string str{"Laura Chambers"};

    mte::bus::Sender<std::string> test_sender{"test_channel_1"};
    mte::bus::Receiver<std::string> test_receiver{"test_channel_1"};

    // Wait 1 second to work around the slow joiner problem.
    std::this_thread::sleep_for(std::chrono::seconds{1});
    test_sender.Send(str);

    // Wait 1 more second to ensure the I/O thread actually sends the message.
    std::this_thread::sleep_for(std::chrono::seconds{1});
    const auto received = test_receiver.ReceiveLatest();

    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(str, received.value());
}

TEST(BusTests, ReceiveLatestReceivesLatest) {
    const std::string str_1{"Laura Chambers"};
    const std::string str_2{"Jonathan Parson"};

    mte::bus::Sender<std::string> test_sender{"test_channel_1"};
    mte::bus::Receiver<std::string> test_receiver{"test_channel_1"};

    // Wait 1 second to work around the slow joiner problem.
    std::this_thread::sleep_for(std::chrono::seconds{1});
    test_sender.Send(str_1);
    test_sender.Send(str_2);

    // Wait 1 more second to ensure the I/O thread actually sends the messages.
    std::this_thread::sleep_for(std::chrono::seconds{1});
    const auto received = test_receiver.ReceiveLatest();

    ASSERT_TRUE(received.has_value());
    EXPECT_EQ(str_2, received.value());
}

TEST(BusTests, ReceiveLatestDoesNotBlock) {
    mte::bus::Receiver<std::string> test_receiver{"test_channel_1"};
    const auto received = test_receiver.ReceiveLatest();

    EXPECT_FALSE(received.has_value());
}

TEST(BusTests, ReceiveMultipleReceivesMultiple) {
    const std::string str_1{"Laura Chambers"};
    const std::string str_2{"Jonathan Parson"};
    const std::string str_3{"Taylor Robertson"};

    mte::bus::Sender<std::string> test_sender{"test_channel_1"};
    mte::bus::Receiver<std::string> test_receiver{"test_channel_1"};

    // Wait 1 second to work around the slow joiner problem.
    std::this_thread::sleep_for(std::chrono::seconds{1});
    test_sender.Send(str_1);
    test_sender.Send(str_2);
    test_sender.Send(str_3);

    // Wait 1 more second to ensure the I/O thread actually sends the messages.
    std::this_thread::sleep_for(std::chrono::seconds{1});
    const auto received = test_receiver.ReceiveMultiple(2);

    ASSERT_EQ(2, received.size());
    EXPECT_EQ(str_1, received.at(0));
    EXPECT_EQ(str_2, received.at(1));
}

TEST(BusTests, ReceiveMultipleDoesNotBlock) {
    mte::bus::Receiver<std::string> test_receiver{"test_channel_1"};
    const auto received = test_receiver.ReceiveMultiple(10);

    EXPECT_TRUE(received.empty());
}
