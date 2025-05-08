//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2016 KONDO Takatoshi
//
//    Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//    http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef MSGPACK_DEFINE_DECL_HPP
#define MSGPACK_DEFINE_DECL_HPP

// BOOST_PP_VARIADICS is defined in boost/preprocessor/config/config.hpp
// http://www.boost.org/libs/preprocessor/doc/ref/variadics.html
// However, supporting compiler detection is not complete. msgpack-c requires
// variadic macro arguments support. So BOOST_PP_VARIADICS is defined here explicitly.
#if !defined(MSGPACK_PP_VARIADICS)
#define MSGPACK_PP_VARIADICS
#endif

#include <rpc/msgpack/preprocessor.hpp>

#include "rpc/msgpack/versioning.hpp"

// for MSGPACK_ADD_ENUM
#include "rpc/msgpack/adaptor/int.hpp"

#define MSGPACK_DEFINE_ARRAY(...) \
    template <typename Packer> \
    void msgpack_pack(Packer& pk) const \
    { \
        clmdep_msgpack::type::make_define_array(__VA_ARGS__).msgpack_pack(pk); \
    } \
    void msgpack_unpack(clmdep_msgpack::object const& o) \
    { \
        clmdep_msgpack::type::make_define_array(__VA_ARGS__).msgpack_unpack(o); \
    }\
    template <typename MSGPACK_OBJECT> \
    void msgpack_object(MSGPACK_OBJECT* o, clmdep_msgpack::zone& z) const \
    { \
        clmdep_msgpack::type::make_define_array(__VA_ARGS__).msgpack_object(o, z); \
    }

#define MSGPACK_BASE_ARRAY(base) (*const_cast<base *>(static_cast<base const*>(this)))
#define MSGPACK_NVP(name, value) (name) (value)

#define MSGPACK_DEFINE_MAP_EACH_PROC(r, data, elem) \
    MSGPACK_PP_IF( \
        MSGPACK_PP_IS_BEGIN_PARENS(elem), \
        elem, \
        (MSGPACK_PP_STRINGIZE(elem))(elem) \
    )

#define MSGPACK_DEFINE_MAP_IMPL(...) \
    MSGPACK_PP_SEQ_TO_TUPLE( \
        MSGPACK_PP_SEQ_FOR_EACH( \
            MSGPACK_DEFINE_MAP_EACH_PROC, \
            0, \
            MSGPACK_PP_VARIADIC_TO_SEQ(__VA_ARGS__) \
        ) \
    )

#define MSGPACK_DEFINE_MAP(...) \
    template <typename Packer> \
    void msgpack_pack(Packer& pk) const \
    { \
        clmdep_msgpack::type::make_define_map \
            MSGPACK_DEFINE_MAP_IMPL(__VA_ARGS__) \
            .msgpack_pack(pk); \
    } \
    void msgpack_unpack(clmdep_msgpack::object const& o) \
    { \
        clmdep_msgpack::type::make_define_map \
            MSGPACK_DEFINE_MAP_IMPL(__VA_ARGS__) \
            .msgpack_unpack(o); \
    }\
    template <typename MSGPACK_OBJECT> \
    void msgpack_object(MSGPACK_OBJECT* o, clmdep_msgpack::zone& z) const \
    { \
        clmdep_msgpack::type::make_define_map \
            MSGPACK_DEFINE_MAP_IMPL(__VA_ARGS__) \
            .msgpack_object(o, z); \
    }

#define MSGPACK_BASE_MAP(base) \
    (MSGPACK_PP_STRINGIZE(base))(*const_cast<base *>(static_cast<base const*>(this)))

// MSGPACK_ADD_ENUM must be used in the global namespace.
#define MSGPACK_ADD_ENUM(enum_name) \
  namespace clmdep_msgpack { \
  /** @cond */ \
  MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) { \
  /** @endcond */ \
  namespace adaptor { \
    template<> \
    struct convert<enum_name> { \
      clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, enum_name& v) const { \
        clmdep_msgpack::underlying_type<enum_name>::type tmp; \
        clmdep_msgpack::operator>>(o, tmp);                   \
        v = static_cast<enum_name>(tmp);   \
        return o; \
      } \
    }; \
    template<> \
    struct object<enum_name> { \
      void operator()(clmdep_msgpack::object& o, const enum_name& v) const { \
        clmdep_msgpack::underlying_type<enum_name>::type tmp = static_cast<clmdep_msgpack::underlying_type<enum_name>::type>(v); \
        clmdep_msgpack::operator<<(o, tmp);                                    \
      } \
    }; \
    template<> \
    struct object_with_zone<enum_name> { \
      void operator()(clmdep_msgpack::object::with_zone& o, const enum_name& v) const {  \
        clmdep_msgpack::underlying_type<enum_name>::type tmp = static_cast<clmdep_msgpack::underlying_type<enum_name>::type>(v); \
        clmdep_msgpack::operator<<(o, tmp);                                    \
      } \
    }; \
    template <> \
    struct pack<enum_name> { \
      template <typename Stream> \
      clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const enum_name& v) const { \
          return clmdep_msgpack::operator<<(o, static_cast<clmdep_msgpack::underlying_type<enum_name>::type>(v)); \
      } \
    }; \
  } \
  /** @cond */ \
  } \
  /** @endcond */ \
  }

#if defined(MSGPACK_USE_DEFINE_MAP)
#define MSGPACK_DEFINE MSGPACK_DEFINE_MAP
#define MSGPACK_BASE MSGPACK_BASE_MAP
#else  // defined(MSGPACK_USE_DEFINE_MAP)
#define MSGPACK_DEFINE MSGPACK_DEFINE_ARRAY
#define MSGPACK_BASE MSGPACK_BASE_ARRAY
#endif // defined(MSGPACK_USE_DEFINE_MAP)


#include "rpc/msgpack/v1/adaptor/define_decl.hpp"
#include "rpc/msgpack/v2/adaptor/define_decl.hpp"

#endif // MSGPACK_DEFINE_DECL_HPP
