//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2017 KONDO Takatoshi
//
//    Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//    http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef MSGPACK_V1_TYPE_STRING_VIEW_HPP
#define MSGPACK_V1_TYPE_STRING_VIEW_HPP

#if __cplusplus >= 201703

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"
#include "rpc/msgpack/adaptor/check_container_size.hpp"

#include <string_view>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace adaptor {

template <>
struct convert<std::string_view> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, std::string_view& v) const {
        switch (o.type) {
        case clmdep_msgpack::type::BIN:
            v = std::string_view(o.via.bin.ptr, o.via.bin.size);
            break;
        case clmdep_msgpack::type::STR:
            v = std::string_view(o.via.str.ptr, o.via.str.size);
            break;
        default:
            throw clmdep_msgpack::type_error();
            break;
        }
        return o;
    }
};

template <>
struct pack<std::string_view> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const std::string_view& v) const {
        uint32_t size = checked_get_container_size(v.size());
        o.pack_str(size);
        o.pack_str_body(v.data(), size);
        return o;
    }
};

template <>
struct object<std::string_view> {
    void operator()(clmdep_msgpack::object& o, const std::string_view& v) const {
        uint32_t size = checked_get_container_size(v.size());
        o.type = clmdep_msgpack::type::STR;
        o.via.str.ptr = v.data();
        o.via.str.size = size;
    }
};

template <>
struct object_with_zone<std::string_view> {
    void operator()(clmdep_msgpack::object::with_zone& o, const std::string_view& v) const {
        static_cast<clmdep_msgpack::object&>(o) << v;
    }
};


} // namespace adaptor

/// @cond
} // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

} // namespace clmdep_msgpack

#endif // __cplusplus >= 201703

#endif // MSGPACK_V1_TYPE_STRING_VIEW_HPP
