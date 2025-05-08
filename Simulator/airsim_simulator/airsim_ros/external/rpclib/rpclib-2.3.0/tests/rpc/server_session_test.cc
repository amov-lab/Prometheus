#include <chrono>
#include <memory>
#include <future>

#include "gtest/gtest.h"

#include "rpc/client.h"
#include "rpc/server.h"
#include "rpc/this_session.h"
#include "rpc/rpc_error.h"
#include "rpc/detail/make_unique.h"
#include "testutils.h"

using namespace rpc::testutils;

class server_session_test : public testing::Test {
public:
    server_session_test() :
            s("127.0.0.1", test_port),
            c("127.0.0.1", test_port) {
        s.bind("consume_big_param", [](std::string const& str){ (void)str; });
        s.bind("func", [](){ return 0; });
        s.bind("get_sid", [](){ return rpc::this_session().id(); });
        s.async_run();
    }

protected:
    static const int test_port = rpc::constants::DEFAULT_PORT;
    rpc::server s;
    rpc::client c;
};

TEST_F(server_session_test, consume_big_param) {
    std::size_t blob_size = 2 << 10 << 10;
    for (int i = 0; i < 4; ++i) {
        c.call("consume_big_param", get_blob(blob_size));
        blob_size *= 2;
    }
    // no crash is enough
}

TEST_F(server_session_test, connection_closed_properly) {
#ifdef RPCLIB_WIN32
	const unsigned max_tries = 10;
#else
	const unsigned max_tries = 1000;
#endif
    for (unsigned counter = 0; counter < max_tries; ++counter) {
        rpc::client client("localhost", rpc::constants::DEFAULT_PORT);
        auto response = client.call("func");
    }
    // no crash is enough
}

TEST_F(server_session_test, session_id_unique) {
    rpc::client c2("localhost", rpc::constants::DEFAULT_PORT);
    auto sid1 = c.call("get_sid").as<rpc::session_id_t>();
    auto sid2 = c2.call("get_sid").as<rpc::session_id_t>();
    EXPECT_NE(sid1, sid2);
}

TEST(server_session_test_bug153, bug_153_crash_on_client_timeout) {
    rpc::server s("127.0.0.1", rpc::constants::DEFAULT_PORT);
    s.bind("bug_153", []() {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return 0;
    });
    s.async_run();

    auto client = std::unique_ptr<rpc::client>(new rpc::client("localhost", rpc::constants::DEFAULT_PORT));
    client->set_timeout(5);

    try {
        client->call("bug_153");
    } catch(rpc::timeout& ) {
        client.reset();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    // no crash is enough
}


TEST(server_session_test_bug175, bug_175_multhread_crash) {
    const int nr_threads = 4;
    const int nr_calls_per_thread = 100;

    rpc::server s("127.0.0.1", rpc::constants::DEFAULT_PORT);
    s.bind("bug_175", [&](int idx, int i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return 0;
    });
    s.async_run(nr_threads);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto spam_call = [&](int idx, int nr_calls) {
        for (int i=0; i<nr_calls; ++i) {
            auto client = std::unique_ptr<rpc::client>(new rpc::client("localhost", rpc::constants::DEFAULT_PORT));
            client->call("bug_175", idx, i);
        }
    };

    std::vector<std::future<void>> futures;
    for (int i=0; i<nr_threads; ++i)
        futures.push_back(std::async(std::launch::async, spam_call, i, nr_calls_per_thread));

    for (auto& future: futures)
        future.get();

    s.stop();
}
