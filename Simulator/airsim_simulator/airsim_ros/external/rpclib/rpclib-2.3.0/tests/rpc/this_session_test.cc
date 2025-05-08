#include <chrono>
#include <thread>

#include "gtest/gtest.h"

#include "rpc/client.h"
#include "rpc/server.h"
#include "rpc/this_session.h"

#include "testutils.h"

using namespace rpc::testutils;
using namespace rpc;

static RPCLIB_CONSTEXPR uint16_t test_port = rpc::constants::DEFAULT_PORT;

class this_session_test : public testing::Test {
public:
    this_session_test() : s(test_port) {}

protected:
    rpc::server s;
};

TEST_F(this_session_test, post_exit) {
    s.bind("exit", []() { rpc::this_session().post_exit(); });
    s.async_run();

    rpc::client c("127.0.0.1", test_port);
    c.call("exit");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto f = c.async_call("exit");
    EXPECT_EQ(f.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
    EXPECT_EQ(c.get_connection_state(), client::connection_state::disconnected);
}

TEST_F(this_session_test, post_exit_specific_to_session) {
    s.bind("exit", [](bool do_exit) {
        if (do_exit) {
            rpc::this_session().post_exit();
        }
    });
    s.async_run();

    rpc::client c("127.0.0.1", test_port);
    rpc::client c2("127.0.0.1", test_port);
    c2.call("exit", false);
    c.call("exit", true);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto f = c.async_call("exit");
    c2.call("exit", false);
    EXPECT_EQ(f.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
    EXPECT_EQ(c.get_connection_state(), client::connection_state::disconnected);
    EXPECT_EQ(c2.get_connection_state(), client::connection_state::connected);
}
