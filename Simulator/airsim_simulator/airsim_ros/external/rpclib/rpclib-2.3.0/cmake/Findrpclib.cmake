#
# Findrpclib.cmake can be distributed with the source of
# your software that depends on rpc. This allows you
# to write
#
#     find_package(rpclib)
#
# in your CMakeLists.txt.
#
# The script sets the following variables:
#
#   * RPCLIB_FOUND: true if the rpc headers and libraries were found
#   * RPCLIB_INCLUDE_DIR: the directory where rpc includes are located.
#                         This means #include "rpc/server.h" works if
#                         you add this directory, not #include "server.h".
#   * RPCLIB_LIBS: The static libraries of rpc (for now, this is only one
#                  library, but plural was chosen to be future-proof).
#   * RPCLIB_EXTRA_FLAGS: Extra flags that need to be added to the C++ compiler
#                         flags (e.g. CMAKE_CXX_FLAGS)
#   * RPCLIB_EXTRA_FLAGS_DEBUG: Same as above, but for debug configuration.
#                               (e.g. CMAKE_CXX_FLAGS_DEBUG)
#
# For finding in custom locations, you may set RPCLIB_ROOT as a cmake variable or
# environment variable.
#
# Greatly inspired by FindSFML.cmake
# (https://github.com/SFML/SFML/blob/master/cmake/Modules/FindSFML.cmake)
#

set(FIND_RPCLIB_PATHS
    ${RPCLIB_ROOT}
    $ENV{RPCLIB_ROOT}
    ${CMAKE_CURRENT_LIST_DIR}/..                   # To support in-tree build
    ${CMAKE_CURRENT_LIST_DIR}/../build/output/lib  #
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local
    /usr
    /opt/local
    /opt)

find_path(RPCLIB_INCLUDE_DIR
    PATH_SUFFIXES "include"
    NAMES "rpc/version.h"
    PATHS ${FIND_RPCLIB_PATHS})

find_library(RPCLIB_LIBS
    NAMES librpc rpclib rpc
    PATHS ${FIND_RPCLIB_PATHS})

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

set(RPCLIB_COMPILE_DEFINITIONS
    "ASIO_STANDALONE"
    "RPCLIB_ASIO=clmdep_asio"
    "RPCLIB_FMT=clmdep_fmt"
    "RPCLIB_MSGPACK=clmdep_msgpack"
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(rpclib
                      FOUND_VAR RPCLIB_FOUND
                      REQUIRED_VARS RPCLIB_INCLUDE_DIR RPCLIB_LIBS
                      VERSION_VAR RPCLIB_VERSION_STRING)

