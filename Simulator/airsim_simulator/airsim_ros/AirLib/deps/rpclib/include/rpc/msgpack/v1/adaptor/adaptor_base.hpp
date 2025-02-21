//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2015-2016 KONDO Takatoshi
//
//    Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//    http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef MSGPACK_V1_ADAPTOR_BASE_HPP
#define MSGPACK_V1_ADAPTOR_BASE_HPP

#include "rpc/msgpack/v1/adaptor/adaptor_base_decl.hpp"

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond


namespace adaptor {

// Adaptor functors

template <typename T, typename Enabler>
struct convert {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, T& v) const;
};

template <typename T, typename Enabler>
struct pack {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, T const& v) const;
};

template <typename T, typename Enabler>
struct object {
    void operator()(clmdep_msgpack::object& o, T const& v) const;
};

template <typename T, typename Enabler>
struct object_with_zone {
    void operator()(clmdep_msgpack::object::with_zone& o, T const& v) const;
};

} // namespace adaptor

// operators

template <typename T>
inline
typename clmdep_msgpack::enable_if<
    !is_array<T>::value,
    clmdep_msgpack::object const&
>::type
operator>> (clmdep_msgpack::object const& o, T& v) {
    return clmdep_msgpack::adaptor::convert<T>()(o, v);
}
template <typename T, std::size_t N>
inline
clmdep_msgpack::object const& operator>> (clmdep_msgpack::object const& o, T(&v)[N]) {
    return clmdep_msgpack::adaptor::convert<T[N]>()(o, v);
}

template <typename Stream, typename T>
inline
typename clmdep_msgpack::enable_if<
    !is_array<T>::value,
    clmdep_msgpack::packer<Stream>&
>::type
operator<< (clmdep_msgpack::packer<Stream>& o, T const& v) {
    return clmdep_msgpack::adaptor::pack<T>()(o, v);
}
template <typename Stream, typename T, std::size_t N>
inline
clmdep_msgpack::packer<Stream>& operator<< (clmdep_msgpack::packer<Stream>& o, const T(&v)[N]) {
    return clmdep_msgpack::adaptor::pack<T[N]>()(o, v);
}

template <typename T>
inline
typename clmdep_msgpack::enable_if<
    !is_array<T>::value
>::type
operator<< (clmdep_msgpack::object& o, T const& v) {
    clmdep_msgpack::adaptor::object<T>()(o, v);
}
template <typename T, std::size_t N>
inline
void operator<< (clmdep_msgpack::v1::object& o, const T(&v)[N]) {
    clmdep_msgpack::v1::adaptor::object<T[N]>()(o, v);
}

template <typename T>
inline
typename clmdep_msgpack::enable_if<
    !is_array<T>::value
>::type
operator<< (clmdep_msgpack::object::with_zone& o, T const& v) {
    clmdep_msgpack::adaptor::object_with_zone<T>()(o, v);
}
template <typename T, std::size_t N>
inline
void operator<< (clmdep_msgpack::object::with_zone& o, const T(&v)[N]) {
    clmdep_msgpack::adaptor::object_with_zone<T[N]>()(o, v);
}

/// @cond
} // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

} // namespace clmdep_msgpack


#endif // MSGPACK_V1_ADAPTOR_BASE_HPP
