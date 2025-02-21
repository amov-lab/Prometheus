Compiling `rpclib` is a fairly normal and unsurprising experience if you have used cmake-based projects. This document details the advanced building options as well as building the documentation which you might find useful if you want to reproduce everything locally.

## Default configuration

The default configuration is the one intended for end-users. If you want to hack on `rpclib`, you might be interested in some of the Advanced options.

## Compiling

This includes most (all?) Linux distributions, cygwin and Windows. Building `rpclib` is very similar to other cmake-based projects:

```
git clone git@github.com:rpclib/rpclib.git
cd rpclib
mkdir build
cd build
cmake ..
cmake --build .
```

And that's it. If all goes well, your build output will be in the `output` directory.

## Advanced options

There are some compilation options that affect the build output. These options can be set using ccmake, cmake-gui or on the cmake command line.

| Name | Default value | Usage
|------|---------------|------
|`RPCLIB_BUILD_TESTS` | OFF |Builds the unit tests of the library. You might want to turn this on if you are using an unreleased version.
|`RPCLIB_GENERATE_COMPDB` | OFF | Generates a json compilation database for use with clang-based tools (such as clang-tidy, YCM etc.)
|`RPCLIB_BUILD_EXAMPLES` | ON | Builds the collection of example programs that demonstrate the features of `rpclib`.
| `RPCLIB_ENABLE_LOGGING` | OFF |Enables the internal logging of `rpclib`. This slightly affects performance. Currently the logging is not very configurable (for example, everything goes to stdout), but there are plans to make it easier to integrate with your application. Use this feature for debugging purposes.
|`RPCLIB_ENABLE_COVERAGE` | OFF | This enables passing the code coverage generation flag when building with g++. It is used on Travis to provide coverage monitoring in tandem with Coveralls.io.
