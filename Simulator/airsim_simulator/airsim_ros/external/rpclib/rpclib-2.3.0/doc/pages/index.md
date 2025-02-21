# rpclib - modern msgpack-rpc for C++

Welcome! `rpclib` is a msgpack-rpc library written using modern C++.
The goal of this library is to provide a simple, no-nonsense RPC solution.
`rpclib` is compatible with C++11 and C++14 (currently the difference is that it compiles slightly
faster in C++14 mode).

## What does it look like?

Here is a very simple example, a kind of hello world for network-related libraries, the "echo"
server and client:

```cpp
#include "rpc/server.h"
#include <string>
using std::string;

int main() {
  rpc::server srv(8080);

  srv.bind("echo", [](string const& s) {
    return string("> ") + s;
  });

  srv.run();
  return 0;
}
```

```cpp
#include "rpc/client.h"
#include <iostream>
#include <string>
using std::string;

int main() {
  rpc::client c("localhost", 8080);

  string input, result;
  while (std::getline(std::cin, input)) {
    if (!input.empty()) {
      result = c.call("echo", input).as<string>();
      std::cout << result << std::endl;
    }
  }
}
```

## Featured chapters

  * [Getting started](gettingstarted.md) - How to set up your environment to start using `rpclib`
  * [Primer](primer.md) - A longer tutorial introduction to using the library, intended for newcomers
  * [Cookbook](cookbook.md) - Lots of examples of common tasks the library was designed for handling

