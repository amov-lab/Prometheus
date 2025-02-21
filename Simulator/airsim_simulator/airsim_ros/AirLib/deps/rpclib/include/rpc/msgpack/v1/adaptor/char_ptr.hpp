//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2014-2015 KONDO Takatoshi
//
//    Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//    http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef MSGPACK_V1_TYPE_CHAR_PTR_HPP
#define MSGPACK_V1_TYPE_CHAR_PTR_HPP

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/object_fwd.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"
#include "rpc/msgpack/adaptor/check_container_size.hpp"

#include <cstring>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace adaptor {

template <>
struct pack<const char*> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const char* v) const {
        uint32_t size = checked_get_container_size(std::strlen(v));
        o.pack_str(size);
        o.pack_str_body(v, size);
        return o;
    }
};

template <>
struct object_with_zone<const char*> {
    void operator()(clmdep_msgpack::object::with_zone& o, const char* v) const {
        uint32_t size = checked_get_container_size(std::strlen(v));
        o.type = clmdep_msgpack::type::STR;
        char* ptr = static_cast<char*>(o.zone.allocate_align(size, MSGPACK_ZONE_ALIGNOF(char)));
        o.via.str.ptr = ptr;
        o.via.str.size = size;
        std::memcpy(ptr, v, size);
    }
};

template <>
struct object<const char*> {
    void operator()(clmdep_msgpack::object& o, const char* v) const {
        uint32_t size = checked_get_container_size(std::strlen(v));
        o.type = clmdep_msgpack::type::STR;
        o.via.str.ptr = v;
        o.via.str.size = size;
    }
};


template <>
struct pack<char*> {
    template <typename Stream>
    packer<Stream>& operator()(packer<Stream>& o, char* v) const {
        return o << static_cast<const char*>(v);
    }
};

template <>
struct object_with_zone<char*> {
    void operator()(clmdep_msgpack::object::with_zone& o, char* v) const {
        o << static_cast<const char*>(v);
    }
};

template <>
struct object<char*> {
    void operator()(clmdep_msgpack::object& o, char* v) const {
        o << static_cast<const char*>(v);
    }
};

} // namespace adaptor

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

}  // namespace clmdep_msgpack

#endif // MSGPACK_V1_TYPE_CHAR_PTR_HPP
