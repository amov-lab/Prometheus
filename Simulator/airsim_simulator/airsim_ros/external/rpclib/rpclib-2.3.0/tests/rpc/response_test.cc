#include "gtest/gtest.h"

#include "testutils.h"
#include <atomic>
#include <memory>
#include <thread>

#include "rpc/detail/response.h"

using namespace rpc::testutils;
using namespace rpc::detail;

TEST(response, object_ctor) {
    auto o = make_unpacked(3, 42, "foo", "bar");
    response r(std::move(o));
    EXPECT_EQ(r.get_id(), 42);
    std::string error = r.get_error()->as<std::string>();
    EXPECT_TRUE(error == "foo");

    std::string result = r.get_result()->as<std::string>();
    EXPECT_TRUE(result == "bar");
}

TEST(response, writing) {
    auto obj = make_unpacked(1, 42, "foo", "bar");
    response r(std::move(obj));
    auto buf1 = r.get_data();
    RPCLIB_MSGPACK::zone z;
    response::response_type same_obj(1, 42, 
                                     RPCLIB_MSGPACK::object("foo", z),
                                     RPCLIB_MSGPACK::object("bar", z));
    RPCLIB_MSGPACK::sbuffer buf2;
    RPCLIB_MSGPACK::pack(buf2, same_obj);

    EXPECT_EQ(buf1.size(), buf2.size());
    EXPECT_EQ(0, memcmp(buf2.data(), buf2.data(), buf1.size()));
}
