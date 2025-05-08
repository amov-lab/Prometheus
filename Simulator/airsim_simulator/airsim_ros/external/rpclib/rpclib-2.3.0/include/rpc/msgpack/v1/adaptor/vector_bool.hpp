//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2015 KONDO Takatoshi
//
//    Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//    http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef MSGPACK_V1_TYPE_VECTOR_BOOL_HPP
#define MSGPACK_V1_TYPE_VECTOR_BOOL_HPP

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/object_fwd.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"

#include <vector>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace adaptor {

template <typename Alloc>
struct convert<std::vector<bool, Alloc> > {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, std::vector<bool, Alloc>& v) const {
        if (o.type != clmdep_msgpack::type::ARRAY) { throw clmdep_msgpack::type_error(); }
        if (o.via.array.size > 0) {
            v.resize(o.via.array.size);
            clmdep_msgpack::object* p = o.via.array.ptr;
            for (typename std::vector<bool, Alloc>::iterator it = v.begin(), end = v.end();
                 it != end;
                 ++it) {
                *it = p->as<bool>();
                ++p;
            }
        }
        return o;
    }
};

template <typename Alloc>
struct pack<std::vector<bool, Alloc> > {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const std::vector<bool, Alloc>& v) const {
        uint32_t size = checked_get_container_size(v.size());
        o.pack_array(size);
        for(typename std::vector<bool, Alloc>::const_iterator it(v.begin()), it_end(v.end());
            it != it_end; ++it) {
            o.pack(static_cast<bool>(*it));
        }
        return o;
    }
};

template <typename Alloc>
struct object_with_zone<std::vector<bool, Alloc> > {
    void operator()(clmdep_msgpack::object::with_zone& o, const std::vector<bool, Alloc>& v) const {
        o.type = clmdep_msgpack::type::ARRAY;
        if(v.empty()) {
            o.via.array.ptr = MSGPACK_NULLPTR;
            o.via.array.size = 0;
        } else {
            uint32_t size = checked_get_container_size(v.size());
            clmdep_msgpack::object* p = static_cast<clmdep_msgpack::object*>(o.zone.allocate_align(sizeof(clmdep_msgpack::object)*size, MSGPACK_ZONE_ALIGNOF(clmdep_msgpack::object)));
            clmdep_msgpack::object* const pend = p + size;
            o.via.array.ptr = p;
            o.via.array.size = size;
            typename std::vector<bool, Alloc>::const_iterator it(v.begin());
            do {
                *p = clmdep_msgpack::object(static_cast<bool>(*it), o.zone);
                ++p;
                ++it;
            } while(p < pend);
        }
    }
};

} // namespace adaptor

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

}  // namespace clmdep_msgpack

#endif // MSGPACK_V1_TYPE_VECTOR_BOOL_HPP
