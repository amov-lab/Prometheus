
function(rpclib_msvc_support)
  if(MSVC)
    # When building via conan, respect the compilation settings.
    if ("${CONAN_LINK_RUNTIME}" STREQUAL "/MT")
      set(RPCLIB_MSVC_STATIC_RUNTIME ON)
    endif()

    if(RPCLIB_ENABLE_COVERAGE)
      message(FATAL_ERROR "Coverage is only supported with non-MS compilers")
    endif()

    target_compile_definitions(${PROJECT_NAME} PRIVATE
      "WIN32_LEAN_AND_MEAN"
      "NOMINMAX"
      "VC_EXTRALEAN"
      "_CRT_SECURE_NO_WARNINGS"
      "_CRT_NONSTDC_NO_DEPRECATE"
      "_WIN32_WINNT=0x0501"
      "_GNU_SOURCE"
      "ASIO_HAS_STD_ADDRESSOF"
      "ASIO_HAS_STD_ARRAY"
      "ASIO_HAS_CSTDINT"
      "ASIO_HAS_STD_SHARED_PTR"
      "ASIO_HAS_STD_TYPE_TRAITS")

    # MSVC static runtime support
    #
    # While this pollutes global flags (when using add_library), you would not want to
    # build with a disparity anyway. (also, CMake still has no support for this, so you
    # would end up doing something like this yourself).
    if (RPCLIB_MSVC_STATIC_RUNTIME)
      set(variables
        CMAKE_C_FLAGS_DEBUG
        CMAKE_C_FLAGS_MINSIZEREL
        CMAKE_C_FLAGS_RELEASE
        CMAKE_C_FLAGS_RELWITHDEBINFO
        CMAKE_CXX_FLAGS_DEBUG
        CMAKE_CXX_FLAGS_MINSIZEREL
        CMAKE_CXX_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_RELWITHDEBINFO
        )
      message(STATUS
        "MSVC -> forcing use of statically-linked runtime."
        )

      foreach(variable ${variables})
        if(${variable} MATCHES "/MD")
          string(REGEX REPLACE "/MD" "/MT" ${variable} "${${variable}}")
        endif()
      endforeach()

    endif()
  endif()
endfunction()
