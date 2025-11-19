cmake_minimum_required(VERSION 3.14)
project(PathfindingLib VERSION 3.0.0 LANGUAGES CXX)

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
add_library(PathfindingLib INTERFACE)
target_include_directories(PathfindingLib INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# Testing, benchmarking, and examples
option(PATHFINDINGLIB_BUILD_TESTS "Build tests" ON)
option(PATHFINDINGLIB_BUILD_BENCHMARKS "Build benchmarks" ON)
option(PATHFINDINGLIB_BUILD_EXAMPLES "Build examples" ON)

if(PATHFINDINGLIB_BUILD_TESTS OR PATHFINDINGLIB_BUILD_BENCHMARKS)
    include(FetchContent)
endif()

# Google Test
if(PATHFINDINGLIB_BUILD_TESTS)
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
if(PATHFINDINGLIB_BUILD_BENCHMARKS)
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
if(PATHFINDINGLIB_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Installation
include(GNUInstallDirs)
install(TARGETS PathfindingLib
    EXPORT PathfindingLibTargets
)

install(FILES
    PathfindingLib.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/PathfindingLib
)

install(FILES
    classes/Pathfinding_Grid.hpp
    classes/Pathfinding_Node.hpp
    classes/Pathfinding_Types.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/PathfindingLib/classes
)

install(EXPORT PathfindingLibTargets
    FILE PathfindingLibTargets.cmake
    NAMESPACE PathfindingLib::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PathfindingLib
)
