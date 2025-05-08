**[rpclib is looking for maintainers](https://github.com/rpclib/rpclib/issues/273)**

# rpclib ![MIT](https://img.shields.io/badge/license-MIT-blue.svg) [![Build Status](https://travis-ci.org/rpclib/rpclib.svg?branch=master)](https://travis-ci.org/rpclib/rpclib) [![Build status](https://ci.appveyor.com/api/projects/status/9lft2tlamcox8epq?svg=true)](https://ci.appveyor.com/project/sztomi/callme) [![Coverage Status](https://img.shields.io/codecov/c/github/rpclib/rpclib/dev.svg)](https://img.shields.io/codecov/c/github/rpclib/rpclib/dev.svg) ![Coverity](https://scan.coverity.com/projects/7259/badge.svg?flat=1) [![Gitter](https://img.shields.io/gitter/room/nwjs/nw.js.svg?maxAge=2592000)](https://gitter.im/rpclib/Lobby)

`rpclib` is a RPC library for C++, providing both a client and server implementation. It is built using modern C++14, and as such, requires a recent compiler. Main highlights:

  * Expose functions of your program to be called via RPC (from any language
    implementing msgpack-rpc)
  * Call functions through RPC (of programs written in any language)
  * No IDL to learn
  * No code generation step to integrate in your build, just C++

# Look&feel

## Server

```cpp
#include <iostream>
#include "rpc/server.h"

void foo() {
    std::cout << "foo was called!" << std::endl;
}

int main(int argc, char *argv[]) {
    // Creating a server that listens on port 8080
    rpc::server srv(8080);

    // Binding the name "foo" to free function foo.
    // note: the signature is automatically captured
    srv.bind("foo", &foo);

    // Binding a lambda function to the name "add".
    srv.bind("add", [](int a, int b) {
        return a + b;
    });

    // Run the server loop.
    srv.run();

    return 0;
}
```

When `srv.run()` is called, `rpclib` starts the server loop which listens to incoming connections
and tries to dispatch calls to the bound functions. The functions are called from the thread where
`run` was called from. There is also `async_run` that spawns worker threads and returns
immediately.

## Client

```cpp
#include <iostream>
#include "rpc/client.h"

int main() {
    // Creating a client that connects to the localhost on port 8080
    rpc::client client("127.0.0.1", 8080);

    // Calling a function with paramters and converting the result to int
    auto result = client.call("add", 2, 3).as<int>();
    std::cout << "The result is: " << result << std::endl;
    return 0;
}
```

# Status

All planned 1.0.0 features are done and tested; the current state is production-ready.

# Who uses rpclib?

This list is updated as I learn about more people using the library; let me
know if you don't want your project listed here.

  * [Microsoft AirSim](https://github.com/Microsoft/AirSim)

# Thanks

`rpclib` builds on the efforts of fantastic C++ projects. In no particular order:

  * [MessagePack implementation for C and C++](https://github.com/msgpack/msgpack-c) by Takatoshi Kondo ([website](http://msgpack.org/))
  * [asio](https://github.com/chriskohlhoff/asio) by Christopher Kohlhoff ([website](http://think-async.com/Asio))
  * [cppformat](https://github.com/fmtlib/fmt) (now renamed `fmtlib`, by Victor Zverovich ([website](http://fmtlib.net))
  * [googletest](https://github.com/google/googletest) by Google
  * [wheels](https://github.com/rmartinho/wheels) by Martinho Fernandes

Shoutouts to

  * [Appveyor](https://www.appveyor.com/)
  * [Travis CI](https://travis-ci.org)
  * [Coveralls.io](https://coveralls.io/)
  * [Coverity](http://www.coverity.com)
  * [ASan & TSan](https://github.com/google/sanitizers) helped spotting and resolving many bugs.




