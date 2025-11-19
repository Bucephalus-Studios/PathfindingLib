cmake_minimum_required(VERSION 3.14)
project(AStarLib VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable warnings
if(MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# Header-only library
add_library(AStarLib INTERFACE)
target_include_directories(AStarLib INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# Testing, benchmarking, and examples
option(ASTARLIB_BUILD_TESTS "Build tests" ON)
option(ASTARLIB_BUILD_BENCHMARKS "Build benchmarks" ON)
option(ASTARLIB_BUILD_EXAMPLES "Build examples" ON)

if(ASTARLIB_BUILD_TESTS OR ASTARLIB_BUILD_BENCHMARKS)
    include(FetchContent)
endif()

# Google Test
if(ASTARLIB_BUILD_TESTS)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()
    add_subdirectory(tests)
endif()

# Google Benchmark
if(ASTARLIB_BUILD_BENCHMARKS)
    set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        benchmark
        GIT_REPOSITORY https://github.com/google/benchmark.git
        GIT_TAG v1.8.3
    )
    FetchContent_MakeAvailable(benchmark)

    add_subdirectory(benchmarks)
endif()

# Examples
if(ASTARLIB_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Installation
include(GNUInstallDirs)
install(TARGETS AStarLib
    EXPORT AStarLibTargets
)

install(FILES
    AStarLib.hpp
    AStar_Grid.hpp
    AStar_Node.hpp
    AStar_Types.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/AStarLib
)

install(EXPORT AStarLibTargets
    FILE AStarLibTargets.cmake
    NAMESPACE AStarLib::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/AStarLib
)
