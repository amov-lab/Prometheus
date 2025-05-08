//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2014-2015 KONDO Takatoshi
//
//    Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//    http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef MSGPACK_V1_TYPE_VECTOR_CHAR_HPP
#define MSGPACK_V1_TYPE_VECTOR_CHAR_HPP

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"
#include "rpc/msgpack/adaptor/check_container_size.hpp"

#include <vector>
#include <cstring>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace adaptor {

template <typename Alloc>
struct convert<std::vector<char, Alloc> > {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, std::vector<char, Alloc>& v) const {
        switch (o.type) {
        case clmdep_msgpack::type::BIN:
            v.resize(o.via.bin.size);
            if (o.via.bin.size != 0) {
#if (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 7)) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif // (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 7)) && !defined(__clang__)
                std::memcpy(&v.front(), o.via.bin.ptr, o.via.bin.size);
#if (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 7)) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif // (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 7)) && !defined(__clang__)
            }
            break;
        case clmdep_msgpack::type::STR:
            v.resize(o.via.str.size);
            if (o.via.str.size != 0) {
#if (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 7)) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif // (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 7)) && !defined(__clang__)
                std::memcpy(&v.front(), o.via.str.ptr, o.via.str.size);
#if (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 7)) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif // (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 7)) && !defined(__clang__)
            }
            break;
        default:
            throw clmdep_msgpack::type_error();
            break;
        }
        return o;
    }
};

template <typename Alloc>
struct pack<std::vector<char, Alloc> > {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const std::vector<char, Alloc>& v) const {
        uint32_t size = checked_get_container_size(v.size());
        o.pack_bin(size);
        if (size != 0) {
            o.pack_bin_body(&v.front(), size);
        }

        return o;
    }
};

template <typename Alloc>
struct object<std::vector<char, Alloc> > {
    void operator()(clmdep_msgpack::object& o, const std::vector<char, Alloc>& v) const {
        uint32_t size = checked_get_container_size(v.size());
        o.type = clmdep_msgpack::type::BIN;
        if (size != 0) {
            o.via.bin.ptr = &v.front();
        }
        o.via.bin.size = size;
    }
};

template <typename Alloc>
struct object_with_zone<std::vector<char, Alloc> > {
    void operator()(clmdep_msgpack::object::with_zone& o, const std::vector<char, Alloc>& v) const {
        uint32_t size = checked_get_container_size(v.size());
        o.type = clmdep_msgpack::type::BIN;
        o.via.bin.size = size;
        if (size != 0) {
            char* ptr = static_cast<char*>(o.zone.allocate_align(size, MSGPACK_ZONE_ALIGNOF(char)));
            o.via.bin.ptr = ptr;
            std::memcpy(ptr, &v.front(), size);
        }
    }
};

} // namespace adaptor

/// @cond
} // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

} // namespace clmdep_msgpack

#endif // MSGPACK_V1_TYPE_VECTOR_CHAR_HPP
