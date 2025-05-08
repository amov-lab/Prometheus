#include <chrono>
#include <thread>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "rpc/client.h"
#include "rpc/rpc_error.h"
#include "rpc/server.h"
#include "testutils.h"

using namespace rpc::testutils;

static RPCLIB_CONSTEXPR uint16_t test_port = rpc::constants::DEFAULT_PORT;

class server_workers_test : public testing::Test {
public:
    server_workers_test()
        : s("127.0.0.1", test_port), long_count(0), short_count(0) {
        s.bind("long_func", [this]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            ++long_count;
        });
        s.bind("short_func", [this]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            ++short_count;
        });
    }

protected:
    rpc::server s;
    std::atomic_int long_count, short_count;
};

TEST_F(server_workers_test, single_worker) {
    const std::size_t workers = 1;
    s.async_run(workers);
    rpc::client c("127.0.0.1", test_port);
    auto ft_long = c.async_call("long_func");
    auto ft_short = c.async_call("short_func");
    ft_short.wait();

    EXPECT_EQ(1, short_count);
    EXPECT_EQ(1, long_count);

    ft_long.wait();
    EXPECT_EQ(1, short_count);
    EXPECT_EQ(1, long_count);
}

TEST_F(server_workers_test, multiple_workers) {
    const std::size_t workers = 2;
    s.async_run(workers);
    rpc::client c("127.0.0.1", test_port);
    auto ft_long = c.async_call("long_func");
    auto ft_short = c.async_call("short_func");
    ft_short.wait();

    EXPECT_EQ(1, short_count);
    EXPECT_EQ(0, long_count);

    ft_long.wait();
    EXPECT_EQ(1, short_count);
    EXPECT_EQ(1, long_count);
}

class server_error_handling : public testing::Test {
public:
    server_error_handling() : s("127.0.0.1", test_port) {
        s.bind("blue", []() {
            throw std::runtime_error("I'm blue daba dee daba die");
        });
        s.bind("red", []() { throw "Am I evil? Yes I am."; });
        s.async_run();
    }

protected:
    rpc::server s;
};

#ifndef RPCLIB_WIN32
TEST_F(server_error_handling, no_suppress) {
    rpc::client c("127.0.0.1", test_port);
    s.suppress_exceptions(false);
    EXPECT_DEATH({ c.call("blue"); }, "");
    EXPECT_DEATH({ c.call("red"); }, "");
}
#endif

TEST_F(server_error_handling, suppress) {
    s.suppress_exceptions(true);
    rpc::client c("127.0.0.1", test_port);
    // this seems like the opposite check, but the client throwing
    // the exception means that it reached the other side, i.e.
    // the server suppressed it.
    EXPECT_THROW(c.call("blue"), std::runtime_error);
    EXPECT_THROW(c.call("red"), std::runtime_error);
    EXPECT_THROW(c.call("green"), std::runtime_error);
    EXPECT_THROW(c.call("blue", 1), std::runtime_error);
}

TEST_F(server_error_handling, suppress_right_msg) {
    s.suppress_exceptions(true);
    rpc::client c("127.0.0.1", test_port);

    try {
        c.call("blue");
        FAIL() << "There was no exception thrown.";
    } catch (rpc::rpc_error &e) {
        EXPECT_STREQ(e.what(), "rpc::rpc_error during call");
        auto err = e.get_error().as<std::string>();
        EXPECT_TRUE(str_match(err, ".*?I'm blue.*"));
    }

    try {
        c.call("red");
        FAIL() << "There was no exception thrown.";
    } catch (rpc::rpc_error &e) {
        EXPECT_FALSE(str_match(e.what(), ".*?Am I evil.*"));
        auto err = e.get_error().as<std::string>();
        EXPECT_TRUE(str_match(err, ".*?not derived from std::exception.*"));
    }
}

TEST_F(server_error_handling, no_such_method_right_msg) {
    s.suppress_exceptions(true);
    rpc::client c("127.0.0.1", test_port);
    try {
        c.call("green");
        FAIL() << "There was no exception thrown.";
    } catch (rpc::rpc_error &e) {
        auto err = e.get_error().as<std::string>();
        EXPECT_TRUE(str_match(err, ".*?could not find.*"));
    }
}

TEST_F(server_error_handling, wrong_arg_count_void_zeroarg) {
    s.suppress_exceptions(true);
    rpc::client c("127.0.0.1", test_port);
    try {
        c.call("blue", 1);
        FAIL() << "There was no exception thrown.";
    } catch (rpc::rpc_error &e) {
        auto err = e.get_error().as<std::string>();
        EXPECT_TRUE(str_match(err, ".*?invalid number of arguments.*"));
    }
}

class dispatch_unicode : public testing::Test {
public:
    dispatch_unicode()
        : s("127.0.0.1", test_port), str_utf8("árvíztűrő tükörfúrógép") {
        s.bind("utf", [](std::string const &p) { return p; });
        s.async_run();
    }

protected:
    rpc::server s;
    std::string str_utf8;
};

TEST_F(dispatch_unicode, narrow_unicode) {
    rpc::client c("127.0.0.1", test_port);
    EXPECT_EQ(str_utf8, c.call("utf", str_utf8).as<std::string>());
}

TEST(server_misc, single_param_ctor) {
    rpc::server s(test_port);
    s.async_run();
    rpc::client c("127.0.0.1", test_port);
}

TEST(server_misc, server_is_moveable) {
    rpc::server s(test_port);
    s.bind("foo", [](){});
    std::vector<rpc::server> vec;
    vec.push_back(std::move(s));
    EXPECT_THROW(vec[0].bind("foo", [](){}), std::logic_error);
}
