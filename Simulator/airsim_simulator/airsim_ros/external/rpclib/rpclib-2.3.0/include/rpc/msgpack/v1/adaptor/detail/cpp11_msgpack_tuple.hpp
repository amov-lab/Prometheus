//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2008-2016 FURUHASHI Sadayuki and KONDO Takatoshi
//
//    Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//    http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef MSGPACK_V1_CPP11_MSGPACK_TUPLE_HPP
#define MSGPACK_V1_CPP11_MSGPACK_TUPLE_HPP

#include "rpc/msgpack/v1/adaptor/detail/cpp11_msgpack_tuple_decl.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"
#include "rpc/msgpack/pack.hpp"

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace type {

template <class... Args>
inline tuple<Args...> make_tuple(Args&&... args) {
    return tuple<Args...>(std::forward<Args>(args)...);
}

template<class... Args>
inline tuple<Args&&...> forward_as_tuple (Args&&... args) noexcept {
    return tuple<Args&&...>(std::forward<Args>(args)...);
}

template <class... Tuples>
inline auto tuple_cat(Tuples&&... args) ->
    decltype(
        std::tuple_cat(std::forward<typename std::remove_reference<Tuples>::type::base>(args)...)
    ) {
    return std::tuple_cat(std::forward<typename std::remove_reference<Tuples>::type::base>(args)...);
}

template <class... Args>
inline tuple<Args&...> tie(Args&... args) {
    return tuple<Args&...>(args...);
}
} // namespace type

// --- Pack from tuple to packer stream ---
template <typename Stream, typename Tuple, std::size_t N>
struct MsgpackTuplePacker {
    static void pack(
        clmdep_msgpack::packer<Stream>& o,
        const Tuple& v) {
        MsgpackTuplePacker<Stream, Tuple, N-1>::pack(o, v);
        o.pack(type::get<N-1>(v));
    }
};

template <typename Stream, typename Tuple>
struct MsgpackTuplePacker<Stream, Tuple, 1> {
    static void pack (
        clmdep_msgpack::packer<Stream>& o,
        const Tuple& v) {
        o.pack(type::get<0>(v));
    }
};

template <typename Stream, typename Tuple>
struct MsgpackTuplePacker<Stream, Tuple, 0> {
    static void pack (
        clmdep_msgpack::packer<Stream>&,
        const Tuple&) {
    }
};

namespace adaptor {

template <typename... Args>
struct pack<clmdep_msgpack::type::tuple<Args...>> {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(
        clmdep_msgpack::packer<Stream>& o,
        const clmdep_msgpack::type::tuple<Args...>& v) const {
        o.pack_array(sizeof...(Args));
        MsgpackTuplePacker<Stream, decltype(v), sizeof...(Args)>::pack(o, v);
        return o;
    }
};

} // namespace adaptor

// --- Convert from tuple to object ---

template <typename T, typename... Args>
struct MsgpackTupleAsImpl {
    static clmdep_msgpack::type::tuple<T, Args...> as(clmdep_msgpack::object const& o) {
        return clmdep_msgpack::type::tuple_cat(
            clmdep_msgpack::type::make_tuple(o.via.array.ptr[o.via.array.size - sizeof...(Args) - 1].as<T>()),
            MsgpackTupleAs<Args...>::as(o));
    }
};

template <typename... Args>
struct MsgpackTupleAs {
    static clmdep_msgpack::type::tuple<Args...> as(clmdep_msgpack::object const& o) {
        return MsgpackTupleAsImpl<Args...>::as(o);
    }
};

template <>
struct MsgpackTupleAs<> {
    static clmdep_msgpack::type::tuple<> as (clmdep_msgpack::object const&) {
        return clmdep_msgpack::type::tuple<>();
    }
};

template <typename Tuple, std::size_t N>
struct MsgpackTupleConverter {
    static void convert(
        clmdep_msgpack::object const& o,
        Tuple& v) {
        MsgpackTupleConverter<Tuple, N-1>::convert(o, v);
        if (o.via.array.size >= N)
            o.via.array.ptr[N-1].convert<typename std::remove_reference<decltype(type::get<N-1>(v))>::type>(type::get<N-1>(v));
    }
};

template <typename Tuple>
struct MsgpackTupleConverter<Tuple, 1> {
    static void convert (
        clmdep_msgpack::object const& o,
        Tuple& v) {
        o.via.array.ptr[0].convert<typename std::remove_reference<decltype(type::get<0>(v))>::type>(type::get<0>(v));
    }
};

template <typename Tuple>
struct MsgpackTupleConverter<Tuple, 0> {
    static void convert (
        clmdep_msgpack::object const&,
        Tuple&) {
    }
};

namespace adaptor {

template <typename... Args>
struct as<clmdep_msgpack::type::tuple<Args...>, typename std::enable_if<clmdep_msgpack::any_of<clmdep_msgpack::has_as, Args...>::value>::type>  {
    clmdep_msgpack::type::tuple<Args...> operator()(
        clmdep_msgpack::object const& o) const {
        if (o.type != clmdep_msgpack::type::ARRAY) { throw clmdep_msgpack::type_error(); }
        return MsgpackTupleAs<Args...>::as(o);
    }
};

template <typename... Args>
struct convert<clmdep_msgpack::type::tuple<Args...>> {
    clmdep_msgpack::object const& operator()(
        clmdep_msgpack::object const& o,
        clmdep_msgpack::type::tuple<Args...>& v) const {
        if(o.type != clmdep_msgpack::type::ARRAY) { throw clmdep_msgpack::type_error(); }
        MsgpackTupleConverter<decltype(v), sizeof...(Args)>::convert(o, v);
        return o;
    }
};

} // namespace adaptor

// --- Convert from tuple to object with zone ---
template <typename Tuple, std::size_t N>
struct MsgpackTupleToObjectWithZone {
    static void convert(
        clmdep_msgpack::object::with_zone& o,
        const Tuple& v) {
        MsgpackTupleToObjectWithZone<Tuple, N-1>::convert(o, v);
        o.via.array.ptr[N-1] = clmdep_msgpack::object(type::get<N-1>(v), o.zone);
    }
};

template <typename Tuple>
struct MsgpackTupleToObjectWithZone<Tuple, 1> {
    static void convert (
        clmdep_msgpack::object::with_zone& o,
        const Tuple& v) {
        o.via.array.ptr[0] = clmdep_msgpack::object(type::get<0>(v), o.zone);
    }
};

template <typename Tuple>
struct MsgpackTupleToObjectWithZone<Tuple, 0> {
    static void convert (
        clmdep_msgpack::object::with_zone&,
        const Tuple&) {
    }
};

namespace adaptor {

template <typename... Args>
    struct object_with_zone<clmdep_msgpack::type::tuple<Args...>> {
    void operator()(
        clmdep_msgpack::object::with_zone& o,
        clmdep_msgpack::type::tuple<Args...> const& v) const {
        o.type = clmdep_msgpack::type::ARRAY;
        o.via.array.ptr = static_cast<clmdep_msgpack::object*>(o.zone.allocate_align(sizeof(clmdep_msgpack::object)*sizeof...(Args), MSGPACK_ZONE_ALIGNOF(clmdep_msgpack::object)));
        o.via.array.size = sizeof...(Args);
        MsgpackTupleToObjectWithZone<decltype(v), sizeof...(Args)>::convert(o, v);
    }
};

}  // namespace adaptor

/// @cond
}  // MSGPACK_API_VERSION_NAMESPACE(v1)
///@endcond

}  // namespace clmdep_msgpack

#endif // MSGPACK_CPP11_MSGPACK_TUPLE_HPP
