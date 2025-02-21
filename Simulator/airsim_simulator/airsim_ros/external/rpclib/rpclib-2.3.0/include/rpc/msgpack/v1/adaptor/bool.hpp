//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2008-2016 FURUHASHI Sadayuki
//
//    Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//    http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef MSGPACK_V1_TYPE_BOOL_HPP
#define MSGPACK_V1_TYPE_BOOL_HPP

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace adaptor {

template <>
struct convert<bool> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, bool& v) const {
        if(o.type != clmdep_msgpack::type::BOOLEAN) { throw clmdep_msgpack::type_error(); }
        v = o.via.boolean;
        return o;
    }
};

template <>
struct pack<bool> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const bool& v) const {
        if(v) { o.pack_true(); }
        else { o.pack_false(); }
        return o;
    }
};

template <>
struct object<bool> {
    void operator()(clmdep_msgpack::object& o, bool v) const {
        o.type = clmdep_msgpack::type::BOOLEAN;
        o.via.boolean = v;
    }
};

template <>
struct object_with_zone<bool> {
    void operator()(clmdep_msgpack::object::with_zone& o, bool v) const {
        static_cast<clmdep_msgpack::object&>(o) << v;
    }
};

} // namespace adaptor

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

}  // namespace clmdep_msgpack

#endif // MSGPACK_V1_TYPE_BOOL_HPP
