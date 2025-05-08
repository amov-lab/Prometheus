//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2008-2016 FURUHASHI Sadayuki and KONDO Takatoshi
//
//    Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//    http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef MSGPACK_V1_TYPE_NIL_HPP
#define MSGPACK_V1_TYPE_NIL_HPP

#include "rpc/msgpack/v1/adaptor/nil_decl.hpp"

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace type {

struct nil_t { };

inline bool operator<(nil_t const& lhs, nil_t const& rhs) {
    return &lhs < &rhs;
}

inline bool operator==(nil_t const& lhs, nil_t const& rhs) {
    return &lhs == &rhs;
}

}  // namespace type

namespace adaptor {

template <>
struct convert<type::nil_t> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, type::nil_t&) const {
        if(o.type != clmdep_msgpack::type::NIL) { throw clmdep_msgpack::type_error(); }
        return o;
    }
};

template <>
struct pack<type::nil_t> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const type::nil_t&) const {
        o.pack_nil();
        return o;
    }
};

template <>
struct object<type::nil_t> {
    void operator()(clmdep_msgpack::object& o, type::nil_t) const {
        o.type = clmdep_msgpack::type::NIL;
    }
};

template <>
struct object_with_zone<type::nil_t> {
    void operator()(clmdep_msgpack::object::with_zone& o, type::nil_t v) const {
        static_cast<clmdep_msgpack::object&>(o) << v;
    }
};

} // namespace adaptor

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

}  // namespace clmdep_msgpack

#endif // MSGPACK_V1_TYPE_NIL_HPP
