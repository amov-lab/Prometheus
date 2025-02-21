#
# This is a "fake" Findrpclib.cmake. Don't use this in your project,
# it is just an internal hack used by rpclib itself.
# In your project, you will probably want to use the REAL
# Findrpclib.cmake, one directory level up from here!
#

set(RPCLIB_INCLUDE_DIR "${RPCLIB_ROOT_DIR}/include")

if(RPCLIB_INCLUDE_DIR)
    file(READ
        "${RPCLIB_INCLUDE_DIR}/rpc/version.h"
        RPCLIB_VERSION_CONTENTS)
    string(REGEX REPLACE
        ".*#define RPCLIB_VERSION_MAJOR ([0-9]+).*" "\\1"
        RPCLIB_VERSION_MAJOR "${RPCLIB_VERSION_CONTENTS}")
    string(REGEX REPLACE
        ".*#define RPCLIB_VERSION_MINOR ([0-9]+).*" "\\1"
        RPCLIB_VERSION_MINOR "${RPCLIB_VERSION_CONTENTS}")
    string(REGEX REPLACE
        ".*#define RPCLIB_VERSION_PATCH ([0-9]+).*" "\\1"
        RPCLIB_VERSION_PATCH "${RPCLIB_VERSION_CONTENTS}")
    set(RPCLIB_VERSION_STR
        "${RPCLIB_VERSION_MAJOR}.${RPCLIB_VERSION_MINOR}.${RPCLIB_VERSION_PATCH}")
endif()

if (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    set(RPCLIB_EXTRA_FLAGS "-pthread")
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    set(RPCLIB_EXTRA_FLAGS "-pthread")
elseif (${CMAKE_CXX_COMPILER_ID} STREQUAL "MSVC")
    set(RPCLIB_EXTRA_FLAGS "/EHsc")
    set(RPCLIB_EXTRA_FLAGS_DEBUG "/Zi")
endif()

set(RPCLIB_FOUND "1")
set(RPCLIB_LIBS "${RPCLIB_PROJECT_NAME}")

if(NOT $ENV{RPCLIB_DEFAULT_PORT})
    set(RPCLIB_DEFAULT_PORT 8080)
else()
    set(RPCLIB_DEFAULT_PORT $ENV{RPCLIB_DEFAULT_PORT})
endif()

set(RPCLIB_COMPILE_DEFINITIONS
    "ASIO_STANDALONE"
    "RPCLIB_ASIO=clmdep_asio"
    "RPCLIB_FMT=clmdep_fmt"
    "RPCLIB_MSGPACK=clmdep_msgpack"
    "RPCLIB_CXX_STANDARD=14"
    "RPCLIB_DEFAULT_PORT=${RPCLIB_DEFAULT_PORT}"
)
