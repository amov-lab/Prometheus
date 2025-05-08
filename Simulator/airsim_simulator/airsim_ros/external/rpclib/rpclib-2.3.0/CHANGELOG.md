# Changelog

### 2.3.0

This release fixes various issues.

*Fixes*

  * Fix compile error on gcc 4.9
  * Fix warnings in clang 7
  * Fix self-assignment
  * Fix early destruction of server sessions
  * Fix crashes in multithreaded environment (#175)

*Additions*

  * Support calling `rpc::this_server().stop()` from a server procedure (#187)
  * Make rpclib compatible with codebases that do not use exceptions
  * Add `server::port()` to query the port used by the server
  * Set `reuseaddress` option on the server


### 2.2.1

This release fixed a crash on Windows.

*Fixes*:

  * Fixed client crashing when `suppress_exceptions` was on
  and the server threw an exception.

### 2.2.0

This release fixed a number of long-standing issues.

*Fixes*:

  * Fixed macOS build (#142)
  * Updated msgpack (#152)
  * Fixed a bug where the server could crash if the client timed out (#153)
  * Fixed code coverage (moved away from coveralls.io)

*Additions*:

  * Simplified and modularized CMake script (#94)
  * Added `this_session()->id()` which is a unique, per-session id
  * Added waffle.io badge
  * Added missing client::clear_timeout function

### 2.1.1

This release fixes an embarrassing build breakage.

Released on 2017-07-15.

Changes since 2.1.1:

  * Fixes building with coverage using clang


### 2.1.0

This is mainly a bugfix release.

Released on 2017-07-15.

Changes since 2.0.1:

  * Fixed an issue where the server did not properly release closed connections (#125)
  * Fixed issues related to timeouts (#114, #115, #116)
  * There is no longer a default timeout
  * Fixed warnings when compiling with clang
  * Fixed the dispatcher silently accepting multiple functions with the same name (#128)
  * Changed minimum support g++ version to 4.8 from 4.9 (thanks to reddit user *attomsk* for letting me know)

### 2.0.1

This is minor release that does not affect the library itself, just the version
number constants that were forgot in 2.0.0. The documentation has some minor updates
and a huge change in looks.

Released on 2017-04-19.

Changes since 2.0.0:

  * Bumped version number
  * Updated documentation looks


### 2.0.0

This is an interface-breaking release (hence the major version number increase).

Released on 2017-04-02.

Changes since 1.0.0:

  * **Added C++11 compatibility** (huge thanks to Github user mbr0wn!)
  * Added global timeout for blocking calls in the client
  * Replaced the internal pimpl_ptr with std::unique_ptr for better stability
  * Fixed a build problem with newer clang versions
  * Contains some preliminary VS2013 work (but it's very far from ready)


### 1.0.0

This is the first, complete release of rpclib.

Released on 2017-03-11.

Changes since 1.0.0-preview:

  * A buffer overflow bug was fixed in the server
  * New unit tests were added
  * CMake no longer assumes libc++ when clang is used
  * Documentation fixes
  * Added the ability to pass extra flags to the build without changing the
    CMakeLists.txt
  * Fixed requiring RPCLIB\_MSGPACK macro when not using Findrpclib.cmake
  * Created [conan](https://conan.io) package
  * A [benchmark suite](https://github.com/rpclib/benchmarks) was implemented

### 1.0.0-preview1

This is the first public release of rpclib! This release is a preview which means that the actual 1.0.0 will be similar, but hopefully improved. Any feedback is welcome!

Released on 2016-08-28.
