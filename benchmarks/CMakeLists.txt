add_executable(benchmarks
    benchmark_pathfinding.cpp
)

target_link_libraries(benchmarks
    PRIVATE
        PathfindingLib
        benchmark::benchmark
)
