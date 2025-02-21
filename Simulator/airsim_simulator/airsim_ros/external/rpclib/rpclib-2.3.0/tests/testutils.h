#pragma once

#ifndef TESTUTILS_H_LHCAMVUX
#define TESTUTILS_H_LHCAMVUX

#include "gmock/gmock.h"
#include "rpc/msgpack.hpp"
#include <regex>
#include <thread>
#include <tuple>

namespace rpc {
namespace testutils {

//! \brief Creates a unpacked messagepack containing its arguments.
template <typename... Types>
inline RPCLIB_MSGPACK::unpacked make_unpacked(Types... items) {
    auto obj = std::make_tuple(items...);
    RPCLIB_MSGPACK::sbuffer buf;
    RPCLIB_MSGPACK::pack(buf, obj);
    return RPCLIB_MSGPACK::unpack(buf.data(), buf.size());
}

//! \brief Creates a packed messagepack containing its arguments and returns the
//! buffer containing it.
template <typename... Types>
inline RPCLIB_MSGPACK::sbuffer make_packed(Types... items) {
    auto obj = std::make_tuple(items...);
    RPCLIB_MSGPACK::sbuffer buf;
    RPCLIB_MSGPACK::pack(buf, obj);
    return buf;
}

inline bool str_match(std::string const &str, std::string const &rgx) {
    using std::regex;
    regex r(rgx);
    return std::regex_match(str, r);
}

struct IDummy {
    virtual void dummy_void_zeroarg() = 0;
    virtual void dummy_void_singlearg(int x) = 0;
    virtual void dummy_void_multiarg(int x, int y) = 0;
};

struct MockDummy : IDummy {
    MOCK_METHOD0(dummy_void_zeroarg, void());
    MOCK_METHOD1(dummy_void_singlearg, void(int));
    MOCK_METHOD2(dummy_void_multiarg, void(int, int));
};

inline std::string get_blob(std::size_t size) {
    std::string s;
    s.resize(size);
    for (auto &c : s) {
        c = static_cast<unsigned char>(rand() % 256);
    }
    return s;
}

}
}

#endif /* end of include guard: TESTUTILS_H_LHCAMVUX */
