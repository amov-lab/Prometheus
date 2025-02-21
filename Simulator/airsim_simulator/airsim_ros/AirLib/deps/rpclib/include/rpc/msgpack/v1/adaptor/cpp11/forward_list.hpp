//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2014 KONDO-2015 Takatoshi
//
//    Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//    http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef MSGPACK_V1_TYPE_CPP11_FORWARD_LIST_HPP
#define MSGPACK_V1_TYPE_CPP11_FORWARD_LIST_HPP

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"
#include "rpc/msgpack/adaptor/check_container_size.hpp"

#include <forward_list>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace adaptor {

template <typename T, typename Alloc>
    struct as<std::forward_list<T, Alloc>, typename std::enable_if<clmdep_msgpack::has_as<T>::value>::type> {
    std::forward_list<T, Alloc> operator()(clmdep_msgpack::object const& o) const {
        if (o.type != clmdep_msgpack::type::ARRAY) { throw clmdep_msgpack::type_error(); }
        std::forward_list<T, Alloc> v;
        clmdep_msgpack::object* p = o.via.array.ptr + o.via.array.size;
        clmdep_msgpack::object* const pend = o.via.array.ptr;
        while (p != pend) {
            --p;
            v.push_front(p->as<T>());
        }
        return v;
    }
};

template <typename T, typename Alloc>
struct convert<std::forward_list<T, Alloc>> {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, std::forward_list<T, Alloc>& v) const {
        if(o.type != clmdep_msgpack::type::ARRAY) { throw clmdep_msgpack::type_error(); }
        v.resize(o.via.array.size);
        clmdep_msgpack::object* p = o.via.array.ptr;
        for (auto &e : v) {
            p->convert(e);
            ++p;
        }
        return o;
    }
};

template <typename T, typename Alloc>
struct pack<std::forward_list<T, Alloc>> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const std::forward_list<T, Alloc>& v) const {
        uint32_t size = checked_get_container_size(std::distance(v.begin(), v.end()));
        o.pack_array(size);
        for(auto const& e : v) o.pack(e);
        return o;
    }
};

template <typename T, typename Alloc>
struct object_with_zone<std::forward_list<T, Alloc>> {
    void operator()(clmdep_msgpack::object::with_zone& o, const std::forward_list<T, Alloc>& v) const {
        o.type = clmdep_msgpack::type::ARRAY;
        if(v.empty()) {
            o.via.array.ptr = MSGPACK_NULLPTR;
            o.via.array.size = 0;
        } else {
            uint32_t size = checked_get_container_size(std::distance(v.begin(), v.end()));
            o.via.array.size = size;
            clmdep_msgpack::object* p = static_cast<clmdep_msgpack::object*>(
                o.zone.allocate_align(sizeof(clmdep_msgpack::object)*size, MSGPACK_ZONE_ALIGNOF(clmdep_msgpack::object)));
            o.via.array.ptr = p;
            for(auto const& e : v) *p++ = clmdep_msgpack::object(e, o.zone);
        }
    }
};

} // namespace adaptor

/// @cond
} // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

} // namespace clmdep_msgpack

#endif // MSGPACK_V1_TYPE_CPP11_FORWARD_LIST_HPP
