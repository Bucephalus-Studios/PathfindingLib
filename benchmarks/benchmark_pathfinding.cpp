#include "../AStarLib.hpp"
#include <benchmark/benchmark.h>
#include <random>

using namespace AStarLib;

// Benchmark simple pathfinding on empty grid
static void BM_PathfindingEmptyGrid_10x10(benchmark::State& state) {
    AStar_Grid<int> grid(10, 10);
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(9, 9);

    for (auto _ : state) {
        auto path = findPath(grid, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(BM_PathfindingEmptyGrid_10x10);

static void BM_PathfindingEmptyGrid_50x50(benchmark::State& state) {
    AStar_Grid<int> grid(50, 50);
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(49, 49);

    for (auto _ : state) {
        auto path = findPath(grid, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(BM_PathfindingEmptyGrid_50x50);

static void BM_PathfindingEmptyGrid_100x100(benchmark::State& state) {
    AStar_Grid<int> grid(100, 100);
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(99, 99);

    for (auto _ : state) {
        auto path = findPath(grid, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(BM_PathfindingEmptyGrid_100x100);

static void BM_PathfindingEmptyGrid_500x500(benchmark::State& state) {
    AStar_Grid<int> grid(500, 500);
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(499, 499);

    for (auto _ : state) {
        auto path = findPath(grid, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(BM_PathfindingEmptyGrid_500x500);

// Benchmark with random obstacles
static void BM_PathfindingWithObstacles_100x100_10Percent(benchmark::State& state) {
    std::mt19937 rng(12345);  // Fixed seed for reproducibility
    std::uniform_int_distribution<int> dist(0, 99);

    AStar_Grid<int> grid(100, 100);

    // Add 10% obstacles
    int obstacleCount = 1000;
    for (int i = 0; i < obstacleCount; ++i) {
        int x = dist(rng);
        int y = dist(rng);
        grid.setObstacle(x, y);
    }

    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(99, 99);

    for (auto _ : state) {
        auto path = findPath(grid, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(BM_PathfindingWithObstacles_100x100_10Percent);

static void BM_PathfindingWithObstacles_100x100_20Percent(benchmark::State& state) {
    std::mt19937 rng(12345);
    std::uniform_int_distribution<int> dist(0, 99);

    AStar_Grid<int> grid(100, 100);

    // Add 20% obstacles
    int obstacleCount = 2000;
    for (int i = 0; i < obstacleCount; ++i) {
        int x = dist(rng);
        int y = dist(rng);
        grid.setObstacle(x, y);
    }

    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(99, 99);

    for (auto _ : state) {
        auto path = findPath(grid, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(BM_PathfindingWithObstacles_100x100_20Percent);

// Benchmark maze-like structures
static void BM_PathfindingMaze_50x50(benchmark::State& state) {
    AStar_Grid<int> grid(50, 50);

    // Create maze-like pattern
    for (int y = 1; y < 49; y += 4) {
        for (int x = 1; x < 49; ++x) {
            if (x % 10 != 5) {  // Leave gaps
                grid.setObstacle(x, y);
            }
        }
    }

    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(49, 49);

    for (auto _ : state) {
        auto path = findPath(grid, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(BM_PathfindingMaze_50x50);

// Benchmark worst case - no path exists
static void BM_PathfindingNoPath_100x100(benchmark::State& state) {
    AStar_Grid<int> grid(100, 100);

    // Create impassable wall
    for (int y = 0; y < 100; ++y) {
        grid.setObstacle(50, y);
    }

    auto start = std::make_tuple(0, 50);
    auto end = std::make_tuple(99, 50);

    for (auto _ : state) {
        auto path = findPath(grid, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(BM_PathfindingNoPath_100x100);

// Benchmark short paths
static void BM_PathfindingShortPath_Adjacent(benchmark::State& state) {
    AStar_Grid<int> grid(10, 10);
    auto start = std::make_tuple(5, 5);
    auto end = std::make_tuple(6, 5);

    for (auto _ : state) {
        auto path = findPath(grid, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(BM_PathfindingShortPath_Adjacent);

static void BM_PathfindingShortPath_5Steps(benchmark::State& state) {
    AStar_Grid<int> grid(10, 10);
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(5, 0);

    for (auto _ : state) {
        auto path = findPath(grid, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(BM_PathfindingShortPath_5Steps);

// Benchmark findPathSimple convenience function
static void BM_FindPathSimple_100x100(benchmark::State& state) {
    std::vector<std::tuple<int, int>> obstacles;
    for (int i = 0; i < 500; ++i) {
        obstacles.push_back({i % 100, (i * 17) % 100});
    }

    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(99, 99);

    for (auto _ : state) {
        auto path = findPathSimple(100, 100, obstacles, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(BM_FindPathSimple_100x100);

// Benchmark Manhattan distance calculation
static void BM_ManhattanDistance(benchmark::State& state) {
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(100, 100);

    for (auto _ : state) {
        auto distance = calculateManhattanDistance(start, end);
        benchmark::DoNotOptimize(distance);
    }
}
BENCHMARK(BM_ManhattanDistance);

// Benchmark grid construction
static void BM_GridConstruction_10x10(benchmark::State& state) {
    for (auto _ : state) {
        AStar_Grid<int> grid(10, 10);
        benchmark::DoNotOptimize(grid);
    }
}
BENCHMARK(BM_GridConstruction_10x10);

static void BM_GridConstruction_100x100(benchmark::State& state) {
    for (auto _ : state) {
        AStar_Grid<int> grid(100, 100);
        benchmark::DoNotOptimize(grid);
    }
}
BENCHMARK(BM_GridConstruction_100x100);

static void BM_GridConstruction_500x500(benchmark::State& state) {
    for (auto _ : state) {
        AStar_Grid<int> grid(500, 500);
        benchmark::DoNotOptimize(grid);
    }
}
BENCHMARK(BM_GridConstruction_500x500);

// Benchmark neighbor retrieval
static void BM_GetNeighbors(benchmark::State& state) {
    AStar_Grid<int> grid(100, 100);
    auto pos = std::make_tuple(50, 50);

    for (auto _ : state) {
        auto neighbors = grid.getNeighbors(pos);
        benchmark::DoNotOptimize(neighbors);
    }
}
BENCHMARK(BM_GetNeighbors);

BENCHMARK_MAIN();
