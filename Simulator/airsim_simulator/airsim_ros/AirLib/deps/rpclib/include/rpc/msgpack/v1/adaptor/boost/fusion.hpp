//
// MessagePack for C++ static resolution routine
//
// Copyright (C) 2015-2016 KONDO Takatoshi
//
//    Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//    http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef MSGPACK_V1_TYPE_BOOST_FUSION_HPP
#define MSGPACK_V1_TYPE_BOOST_FUSION_HPP

#include "rpc/msgpack/versioning.hpp"
#include "rpc/msgpack/adaptor/adaptor_base.hpp"
#include "rpc/msgpack/adaptor/check_container_size.hpp"
#include "rpc/msgpack/meta.hpp"

#include "rpc/msgpack/adaptor/pair.hpp"

#if !defined (MSGPACK_USE_CPP03)
#include "rpc/msgpack/adaptor/cpp11/tuple.hpp"
#endif // #if !defined (MSGPACK_USE_CPP03)

#include <boost/fusion/support/is_sequence.hpp>
#include <boost/fusion/sequence/intrinsic/size.hpp>
#include <boost/fusion/algorithm/iteration/for_each.hpp>
#include <boost/fusion/sequence/intrinsic/at.hpp>
#include <boost/fusion/include/mpl.hpp>
#include <boost/mpl/size.hpp>

namespace clmdep_msgpack {

/// @cond
MSGPACK_API_VERSION_NAMESPACE(v1) {
/// @endcond

namespace adaptor {

namespace detail {

template <typename T>
struct is_std_pair {
    static bool const value = false;
};

template <typename T, typename U>
struct is_std_pair<std::pair<T, U> > {
    static bool const value = true;
};

#if !defined(MSGPACK_USE_CPP03)

template <typename T>
struct is_std_tuple {
    static bool const value = false;
};

template <typename... Args>
struct is_std_tuple<std::tuple<Args...>> {
    static bool const value = true;
};

#endif // !defined(MSGPACK_USE_CPP03)

template <typename T>
struct is_seq_no_pair_no_tuple {
    static bool const value =
        boost::fusion::traits::is_sequence<T>::value
        &&
        !is_std_pair<T>::value
#if !defined (MSGPACK_USE_CPP03)
        &&
        !is_std_tuple<T>::value
#endif // !defined (MSGPACK_USE_CPP03)
        ;
};

} // namespace detail

#if !defined (MSGPACK_USE_CPP03)

template <typename T>
struct as<
    T,
    typename clmdep_msgpack::enable_if<
        detail::is_seq_no_pair_no_tuple<T>::value &&
        boost::mpl::fold<
            T,
            boost::mpl::bool_<true>,
            boost::mpl::if_ <
                boost::mpl::or_<
                    boost::mpl::_1,
                    clmdep_msgpack::has_as<boost::mpl::_2>
                >,
                boost::mpl::bool_<true>,
                boost::mpl::bool_<false>
            >
        >::type::value
    >::type
> {
    T operator()(clmdep_msgpack::object const& o) const {
        if (o.type != clmdep_msgpack::type::ARRAY) { throw clmdep_msgpack::type_error(); }
        if (o.via.array.size != checked_get_container_size(boost::mpl::size<T>::value)) {
            throw clmdep_msgpack::type_error();
        }
        using tuple_t = decltype(to_tuple(std::declval<T>(), gen_seq<boost::mpl::size<T>::value>()));
        return to_t(
            o.as<tuple_t>(),
            clmdep_msgpack::gen_seq<boost::mpl::size<T>::value>());
    }
    template<std::size_t... Is, typename U>
    static std::tuple<
        typename std::remove_reference<
            typename boost::fusion::result_of::at_c<T, Is>::type
        >::type...>
    to_tuple(U const& u, seq<Is...>) {
        return std::make_tuple(boost::fusion::at_c<Is>(u)...);
    }
    template<std::size_t... Is, typename U>
    static T to_t(U const& u, seq<Is...>) {
        return T(std::get<Is>(u)...);
    }
};

#endif // !defined (MSGPACK_USE_CPP03)

template <typename T>
struct convert<T, typename clmdep_msgpack::enable_if<detail::is_seq_no_pair_no_tuple<T>::value>::type > {
    clmdep_msgpack::object const& operator()(clmdep_msgpack::object const& o, T& v) const {
        if (o.type != clmdep_msgpack::type::ARRAY) { throw clmdep_msgpack::type_error(); }
        if (o.via.array.size != checked_get_container_size(boost::fusion::size(v))) {
            throw clmdep_msgpack::type_error();
        }
        uint32_t index = 0;
        boost::fusion::for_each(v, convert_imp(o, index));
        return o;
    }
private:
    struct convert_imp {
        convert_imp(clmdep_msgpack::object const& obj, uint32_t& index):obj_(obj), index_(index) {}
        template <typename U>
        void operator()(U& v) const {
            clmdep_msgpack::adaptor::convert<U>()(obj_.via.array.ptr[index_++], v);
        }
    private:
        clmdep_msgpack::object const& obj_;
        uint32_t& index_;
    };
};

template <typename T>
struct pack<T, typename clmdep_msgpack::enable_if<detail::is_seq_no_pair_no_tuple<T>::value>::type > {
    template <typename Stream>
    clmdep_msgpack::packer<Stream>& operator()(clmdep_msgpack::packer<Stream>& o, const T& v) const {
        uint32_t size = checked_get_container_size(boost::fusion::size(v));
        o.pack_array(size);
        boost::fusion::for_each(v, pack_imp<Stream>(o));
        return o;
    }
private:
    template <typename Stream>
    struct pack_imp {
        pack_imp(clmdep_msgpack::packer<Stream>& stream):stream_(stream) {}
        template <typename U>
        void operator()(U const& v) const {
            stream_.pack(v);
        }
    private:
        clmdep_msgpack::packer<Stream>& stream_;
    };
};

template <typename T>
struct object_with_zone<T, typename clmdep_msgpack::enable_if<detail::is_seq_no_pair_no_tuple<T>::value>::type > {
    void operator()(clmdep_msgpack::object::with_zone& o, const T& v) const {
        uint32_t size = checked_get_container_size(boost::fusion::size(v));
        o.type = clmdep_msgpack::type::ARRAY;
        o.via.array.ptr = static_cast<clmdep_msgpack::object*>(o.zone.allocate_align(sizeof(clmdep_msgpack::object)*size, MSGPACK_ZONE_ALIGNOF(clmdep_msgpack::object)));
        o.via.array.size = size;
        uint32_t count = 0;
        boost::fusion::for_each(v, with_zone_imp(o, count));
    }
private:
    struct with_zone_imp {
        with_zone_imp(clmdep_msgpack::object::with_zone const& obj, uint32_t& count):obj_(obj), count_(count) {}
        template <typename U>
        void operator()(U const& v) const {
            obj_.via.array.ptr[count_++] = clmdep_msgpack::object(v, obj_.zone);
        }
        clmdep_msgpack::object::with_zone const& obj_;
        uint32_t& count_;
    };
};

} // namespace adaptor

/// @cond
} // MSGPACK_API_VERSION_NAMESPACE(v1)
/// @endcond

} // namespace clmdep_msgpack

#endif // MSGPACK_V1_TYPE_BOOST_FUSION_HPP
