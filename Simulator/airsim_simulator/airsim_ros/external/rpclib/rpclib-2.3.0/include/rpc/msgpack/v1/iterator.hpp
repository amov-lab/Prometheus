//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2015-2016 MIZUKI Hirata
//
//    Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//    http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef MSGPACK_V1_ITERATOR_HPP
#define MSGPACK_V1_ITERATOR_HPP
#if !defined(MSGPACK_USE_CPP03)

#include "rpc/msgpack/v1/fbuffer_decl.hpp"

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

inline clmdep_msgpack::object_kv* begin(clmdep_msgpack::object_map &map) { return map.ptr; }
inline const clmdep_msgpack::object_kv* begin(const clmdep_msgpack::object_map &map) { return map.ptr; }
inline clmdep_msgpack::object_kv* end(clmdep_msgpack::object_map &map) { return map.ptr + map.size; }
inline const clmdep_msgpack::object_kv* end(const clmdep_msgpack::object_map &map) { return map.ptr + map.size; }

inline clmdep_msgpack::object* begin(clmdep_msgpack::object_array &array) { return array.ptr; }
inline const clmdep_msgpack::object* begin(const clmdep_msgpack::object_array &array) { return array.ptr; }
inline clmdep_msgpack::object* end(clmdep_msgpack::object_array &array) { return array.ptr + array.size; }
inline const clmdep_msgpack::object* end(const clmdep_msgpack::object_array &array) { return array.ptr + array.size; }

/// @cond
}
/// @endcond

}

#endif // !defined(MSGPACK_USE_CPP03)
#endif // MSGPACK_V1_ITERATOR_HPP
