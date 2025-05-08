include(CheckCXXSourceCompiles)

function(check_warning_flag FLAG VAR)
  set(SRC "int main() {}")
  set(OLD_CMAKE_REQUIRED_FLAGS ${CMAKE_REQUIRED_FLAGS})
  set(CMAKE_REQUIRED_FLAGS "-Werror -Wno-${FLAG}")
  check_cxx_source_compiles(${SRC} ${VAR})
  set(CMAKE_REQUIRED_FLAGS ${OLD_CMAKE_REQUIRED_FLAGS})
endfunction()