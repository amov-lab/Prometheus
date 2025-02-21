#pragma once

#ifndef COMPATIBILITY_H_PODKJ3
#define COMPATIBILITY_H_PODKJ3

#ifndef _MSC_VER

#define RPCLIB_NORETURN [[noreturn]]
#define RPCLIB_CONSTEXPR constexpr
#define RPCLIB_FINAL final

#else

#define RPCLIB_NORETURN __declspec(noreturn)
#define RPCLIB_CONSTEXPR const // bad replacement, but gets the job done
#define RPCLIB_FINAL

#endif // !_MSC_VER

#endif // COMPATIBILITY_H_PODKJ3
