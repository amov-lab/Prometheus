//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2008-2016 FURUHASHI Sadayuki and KONDO Takatoshi
//
//    Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//    http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef MSGPACK_V1_CPP11_DEFINE_MAP_HPP
#define MSGPACK_V1_CPP11_DEFINE_MAP_HPP

#include "rpc/msgpack/v1/adaptor/detail/cpp11_define_map_decl.hpp"

#include <tuple>
#include <map>

namespace clmdep_msgpack {
/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond
namespace type {

template <typename Tuple, std::size_t N>
struct define_map_imp {
    template <typename Packer>
    static void pack(Packer& pk, Tuple const& t) {
        define_map_imp<Tuple, N-1>::pack(pk, t);
        pk.pack(std::get<N-1>(t));
    }
    static void unpack(
        clmdep_msgpack::object const& o, Tuple const& t,
        std::map<std::string, clmdep_msgpack::object const*> const& kvmap) {
        define_map_imp<Tuple, N-2>::unpack(o, t, kvmap);
        auto it = kvmap.find(std::get<N-2>(t));
        if (it != kvmap.end()) {
            it->second->convert(std::get<N-1>(t));
        }
    }
    static void object(clmdep_msgpack::object* o, clmdep_msgpack::zone& z, Tuple const& t) {
        define_map_imp<Tuple, N-2>::object(o, z, t);
        o->via.map.ptr[(N-1)/2].key = clmdep_msgpack::object(std::get<N-2>(t), z);
        o->via.map.ptr[(N-1)/2].val = clmdep_msgpack::object(std::get<N-1>(t), z);
    }
};

template <typename Tuple>
struct define_map_imp<Tuple, 0> {
    template <typename Packer>
    static void pack(Packer&, Tuple const&) {}
    static void unpack(
        clmdep_msgpack::object const&, Tuple const&,
        std::map<std::string, clmdep_msgpack::object const*> const&) {}
    static void object(clmdep_msgpack::object*, clmdep_msgpack::zone&, Tuple const&) {}
};

template <typename... Args>
struct define_map {
    define_map(Args&... args) :
        a(args...) {}
    template <typename Packer>
    void msgpack_pack(Packer& pk) const
    {
        static_assert(sizeof...(Args) % 2 == 0, "");
        pk.pack_map(sizeof...(Args) / 2);

        define_map_imp<std::tuple<Args&...>, sizeof...(Args)>::pack(pk, a);
    }
    void msgpack_unpack(clmdep_msgpack::object const& o) const
    {
        if(o.type != clmdep_msgpack::type::MAP) { throw clmdep_msgpack::type_error(); }
        std::map<std::string, clmdep_msgpack::object const*> kvmap;
        for (uint32_t i = 0; i < o.via.map.size; ++i) {
            kvmap.emplace(
                std::string(
                    o.via.map.ptr[i].key.via.str.ptr,
                    o.via.map.ptr[i].key.via.str.size),
                &o.via.map.ptr[i].val);
        }
        define_map_imp<std::tuple<Args&...>, sizeof...(Args)>::unpack(o, a, kvmap);
    }
    void msgpack_object(clmdep_msgpack::object* o, clmdep_msgpack::zone& z) const
    {
        static_assert(sizeof...(Args) % 2 == 0, "");
        o->type = clmdep_msgpack::type::MAP;
        o->via.map.ptr = static_cast<clmdep_msgpack::object_kv*>(z.allocate_align(sizeof(clmdep_msgpack::object_kv)*sizeof...(Args)/2, MSGPACK_ZONE_ALIGNOF(clmdep_msgpack::object_kv)));
        o->via.map.size = sizeof...(Args) / 2;

        define_map_imp<std::tuple<Args&...>, sizeof...(Args)>::object(o, z, a);
    }

    std::tuple<Args&...> a;
};


template <typename... Args>
inline define_map<Args...> make_define_map(Args&... args)
{
    return define_map<Args...>(args...);
}

}  // namespace type
/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond
}  // namespace clmdep_msgpack

#endif // MSGPACK_V1_CPP11_DEFINE_MAP_HPP
