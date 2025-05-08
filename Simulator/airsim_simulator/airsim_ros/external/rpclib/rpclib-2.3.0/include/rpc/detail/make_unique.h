#pragma once

#ifndef MAKE_UNIQUE_H_FOOBAR
#define MAKE_UNIQUE_H_FOOBAR

#include <memory>

namespace rpc {
namespace detail {

// Default behaviour is to assume C++11, overriding RPCLIB_CXX_STANDARD can use
// newer standards:
#if RPCLIB_CXX_STANDARD >= 14

using std::make_unique;

#else

template<typename T, typename... Ts>
std::unique_ptr<T> make_unique(Ts&&... params)
{
    return std::unique_ptr<T>(new T(std::forward<Ts>(params)...));
}

#endif

} /* detail */
} /* rpc  */

#endif /* end of include guard: MAKE_UNIQUE_H_FOOBAR */

