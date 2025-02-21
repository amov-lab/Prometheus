#pragma once

#ifndef CALL_H_ZXFACADH
#define CALL_H_ZXFACADH

#include <tuple>
#include "rpc/detail/func_tools.h"
#include "rpc/detail/invoke.h"
#include "rpc/detail/is_specialization_of.h"

namespace rpc {
namespace detail {

//! \brief Calls a functor with argument provided directly
template <typename Functor, typename Arg>
auto call(Functor f, Arg &&arg)
    -> decltype(f(std::forward<Arg>(arg)))
{
    return f(std::forward<Arg>(arg));
}


// Default behaviour is to assume C++11, overriding RPCLIB_CXX_STANDARD can use
// newer standards:
#if RPCLIB_CXX_STANDARD >= 14

template <typename Functor, typename... Args, std::size_t... I>
decltype(auto) call_helper(Functor func, std::tuple<Args...> &&params,
                           std::index_sequence<I...>) {
    return func(std::get<I>(params)...);
}

//! \brief Calls a functor with arguments provided as a tuple
template <typename Functor, typename... Args>
decltype(auto) call(Functor f, std::tuple<Args...> &args) {
    return call_helper(f, std::forward<std::tuple<Args...>>(args),
                       std::index_sequence_for<Args...>{});
}

#else

// N is number of arguments left in tuple to unpack

template <size_t N>
struct call_helper
{
    template <typename Functor, typename... ArgsT, typename... ArgsF>
    static auto call(
            Functor f,
            std::tuple<ArgsT...>& args_t,
            ArgsF&&... args_f)
    -> decltype(call_helper<N-1>::call(
                f, args_t, std::get<N-1>(args_t),
                std::forward<ArgsF>(args_f)...))
    {
        return call_helper<N-1>::call(
                f,
                args_t,
                std::get<N-1>(args_t),
                std::forward<ArgsF>(args_f)...
        );
    }
};

template <>
struct call_helper<0>
{
    template <typename Functor, typename... ArgsT, typename... ArgsF>
    static auto call(
            Functor f,
            std::tuple<ArgsT...>&,
            ArgsF&&... args_f)
    -> decltype(f(std::forward<ArgsF>(args_f)...))
    {
        return f(std::forward<ArgsF>(args_f)...);
    }
};

//! \brief Calls a functor with arguments provided as a tuple
template <typename Functor, typename... ArgsT>
auto call(Functor f, std::tuple<ArgsT...>& args_t)
    -> decltype(call_helper<sizeof...(ArgsT)>::call(f, args_t))
{
    return call_helper<sizeof...(ArgsT)>::call(f, args_t);
}

#endif

}
}

#endif /* end of include guard: CALL_H_ZXFACADH */
