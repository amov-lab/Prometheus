//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2015 KONDO Takatoshi
//
//    Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//    http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef MSGPACK_V1_TYPE_BOOST_STRING_REF_HPP
#define MSGPACK_V1_TYPE_BOOST_STRING_REF_HPP

#include <boost/version.hpp>
#if (BOOST_VERSION / 100000) >= 1 && ((BOOST_VERSION / 100) % 1000) >= 53

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"
#include "rpc/msgpack/adaptor/check_container_size.hpp"

#include <boost/utility/string_ref.hpp>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace adaptor {

template <>
struct convert<boost::string_ref> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, boost::string_ref& v) const {
        switch (o.type) {
        case clmdep_msgpack::type::BIN:
            v = boost::string_ref(o.via.bin.ptr, o.via.bin.size);
            break;
        case clmdep_msgpack::type::STR:
            v = boost::string_ref(o.via.str.ptr, o.via.str.size);
            break;
        default:
            throw clmdep_msgpack::type_error();
            break;
        }
        return o;
    }
};

template <>
struct pack<boost::string_ref> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const boost::string_ref& v) const {
        uint32_t size = checked_get_container_size(v.size());
        o.pack_str(size);
        o.pack_str_body(v.data(), size);
        return o;
    }
};

template <>
struct object<boost::string_ref> {
    void operator()(clmdep_msgpack::object& o, const boost::string_ref& v) const {
        uint32_t size = checked_get_container_size(v.size());
        o.type = clmdep_msgpack::type::STR;
        o.via.str.ptr = v.data();
        o.via.str.size = size;
    }
};

template <>
struct object_with_zone<boost::string_ref> {
    void operator()(clmdep_msgpack::object::with_zone& o, const boost::string_ref& v) const {
        static_cast<clmdep_msgpack::object&>(o) << v;
    }
};


} // namespace adaptor

/// @cond
} // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

} // namespace clmdep_msgpack

#endif // (BOOST_VERSION / 100000) >= 1 && ((BOOST_VERSION / 100) % 1000) >= 53

#endif // MSGPACK_V1_TYPE_BOOST_STRING_REF_HPP
