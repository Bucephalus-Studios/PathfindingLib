add_executable(astar_benchmarks
    benchmark_pathfinding.cpp
)

target_link_libraries(astar_benchmarks
    PRIVATE
        AStarLib
        benchmark::benchmark
)
