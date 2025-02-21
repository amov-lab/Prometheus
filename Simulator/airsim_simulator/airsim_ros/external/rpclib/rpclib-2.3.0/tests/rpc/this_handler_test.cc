#include <chrono>

#include "gtest/gtest.h"

#include "rpc/client.h"
#include "rpc/server.h"
#include "rpc/rpc_error.h"
#include "rpc/this_handler.h"

#include "testutils.h"

using namespace rpc::testutils;

static RPCLIB_CONSTEXPR uint16_t test_port = rpc::constants::DEFAULT_PORT;

class this_handler_test : public testing::Test {
public:
    this_handler_test() : s(test_port) {}

protected:
    rpc::server s;
};

TEST_F(this_handler_test, set_error) {
    s.suppress_exceptions(false);
    s.bind("errfunc", []() {
        rpc::this_handler().respond_error("Imma let you finish, but.");
    });
    s.async_run();

    rpc::client c("127.0.0.1", test_port);
    EXPECT_THROW(c.call("errfunc"), std::runtime_error);
    EXPECT_THROW(c.call("errfunc"), rpc::rpc_error);

    try {
        c.call("errfunc");
        FAIL() << "There was no exception thrown.";
    } catch (rpc::rpc_error &e) {
        auto err = e.get_error().as<std::string>();
        EXPECT_TRUE(str_match(err, ".*?Imma let you finish, but.*"));
    }
}

TEST_F(this_handler_test, error_obj) {
    auto err = std::make_tuple(1234, "this is a custom error object");

    s.bind("customerr", [&]() {
        rpc::this_handler().respond_error(err);
    });
    s.async_run();

    rpc::client c("127.0.0.1", test_port);
    EXPECT_THROW(c.call("customerr"), std::runtime_error);
    EXPECT_THROW(c.call("customerr"), rpc::rpc_error);

    try {
        c.call("customerr");
        FAIL() << "There was no exception thrown.";
    } catch (rpc::rpc_error &e) {
        auto err_received = e.get_error().as<std::tuple<int, std::string>>();
        EXPECT_EQ(err_received, err);
    }
}

TEST_F(this_handler_test, set_special_response) {
    std::string text("What? You thought I was a number?");
    s.bind("spec_func", [text](bool special) {
        if (special) {
            rpc::this_handler().respond(text);
        }
        return 5;
    });
    s.async_run();

    rpc::client c("127.0.0.1", test_port);
    EXPECT_EQ(c.call("spec_func", false).as<int>(), 5);
    EXPECT_EQ(c.call("spec_func", true).as<std::string>(), text);
    EXPECT_THROW(c.call("spec_func", true).as<int>(), RPCLIB_MSGPACK::type_error);
}

TEST_F(this_handler_test, disable_response) {
    s.bind("noresp", []() { rpc::this_handler().disable_response(); });
    s.async_run();

    rpc::client c("127.0.0.1", test_port);
    auto f = c.async_call("noresp");
    EXPECT_EQ(f.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
}

TEST_F(this_handler_test, enable_response) {
    s.bind("noresp", []() { rpc::this_handler().disable_response(); });
    s.async_run();

    rpc::client c("127.0.0.1", test_port);
    auto f = c.async_call("noresp");
    EXPECT_EQ(f.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
}
