cmake_minimum_required(VERSION 3.0.0)
project(mandelbrot)

# Find our dependencies
find_package(rpclib REQUIRED)
find_package(SFML COMPONENTS graphics window system)

if(${SFML_FOUND})
        include_directories(${RPCLIB_INCLUDE_DIR} ${SFML_INCLUDE_DIR})

        add_executable(mandelbrot_server mandelbrot_server.cc)
        target_link_libraries(mandelbrot_server ${RPCLIB_LIBS})
        set_target_properties(
                mandelbrot_server
                PROPERTIES
                CXX_STANDARD 14
                COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${RPCLIB_EXTRA_FLAGS}")
        target_compile_definitions(mandelbrot_server PUBLIC ${RPCLIB_COMPILE_DEFINITIONS})

        add_executable(mandelbrot_client mandelbrot_client.cc)
        target_link_libraries(mandelbrot_client ${RPCLIB_LIBS} ${SFML_LIBRARIES})
        set_target_properties(
                mandelbrot_client
                PROPERTIES
                CXX_STANDARD 14
                COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${RPCLIB_EXTRA_FLAGS}")
        target_compile_definitions(mandelbrot_client PUBLIC ${RPCLIB_COMPILE_DEFINITIONS})
endif(${SFML_FOUND})
