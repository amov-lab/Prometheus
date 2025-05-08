function(enable_coverage TARGET)
  if(NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")

    if(APPLE)
      set(COVERAGE_LIB "")
    else()
      set(COVERAGE_LIB -lgcov)
    endif()

    set_target_properties(${TARGET}
      PROPERTIES
      COMPILE_FLAGS --coverage
      LINK_FLAGS    "${COVERAGE_LIB} --coverage"
      )
  endif()
endfunction()
