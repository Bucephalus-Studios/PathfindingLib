#include "../AStarLib.hpp"
#include "obstacle_maps.hpp"
#include <benchmark/benchmark.h>
#include <random>
#include <queue>
#include <unordered_set>
#include <cmath>

using namespace AStarLib;

// ============================================================================
// Pre-generated Grids (initialized once)
// ============================================================================

// Global pre-built grids to avoid re-initialization overhead in benchmarks
AStar_Grid<int> grid_100x100_10pct(100, 100);
AStar_Grid<int> grid_100x100_20pct(100, 100);
AStar_Grid<int> grid_100x100_30pct(100, 100);
bool grids_initialized = false;

void initializeGrids() {
    if (!grids_initialized) {
        // Setup 10% obstacles grid
        for (const auto& obs : ObstacleMaps::obstacles_100x100_10percent) {
            grid_100x100_10pct.setObstacle(std::get<0>(obs), std::get<1>(obs));
        }

        // Setup 20% obstacles grid
        for (const auto& obs : ObstacleMaps::obstacles_100x100_20percent) {
            grid_100x100_20pct.setObstacle(std::get<0>(obs), std::get<1>(obs));
        }

        // Setup 30% obstacles grid
        for (const auto& obs : ObstacleMaps::obstacles_100x100_30percent) {
            grid_100x100_30pct.setObstacle(std::get<0>(obs), std::get<1>(obs));
        }

        grids_initialized = true;
    }
}

// ============================================================================
// Baseline Implementations for Comparison
// ============================================================================

// BFS implementation for comparison
// Uses the same coordinate hashing approach as A* for fair comparison
template <typename CoordType>
std::vector<std::tuple<CoordType, CoordType>> bfs(
    AStar_Grid<CoordType>& grid,
    std::tuple<CoordType, CoordType> start,
    std::tuple<CoordType, CoordType> end) {

    std::queue<std::tuple<CoordType, CoordType>> queue;
    std::unordered_set<std::tuple<CoordType, CoordType>, CoordHash<CoordType>> visited;
    std::unordered_map<std::tuple<CoordType, CoordType>, std::tuple<CoordType, CoordType>,
                       CoordHash<CoordType>> parent;

    queue.push(start);
    visited.insert(start);

    while (!queue.empty()) {
        auto current = queue.front();
        queue.pop();

        if (current == end) {
            // Reconstruct path
            std::vector<std::tuple<CoordType, CoordType>> path;
            auto node = end;
            while (node != start) {
                path.push_back(node);
                node = parent[node];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        auto neighbors = grid.getNeighbors(current);
        for (const auto& neighbor : neighbors) {
            if (visited.find(neighbor) == visited.end()) {
                queue.push(neighbor);
                visited.insert(neighbor);
                parent[neighbor] = current;
            }
        }
    }

    return {};  // No path found
}

// Dijkstra's algorithm implementation for comparison
template <typename CoordType>
std::vector<std::tuple<CoordType, CoordType>> dijkstra(
    AStar_Grid<CoordType>& grid,
    std::tuple<CoordType, CoordType> start,
    std::tuple<CoordType, CoordType> end) {

    using Coord = std::tuple<CoordType, CoordType>;

    auto coordToString = [](const Coord& coord) {
        return std::to_string(std::get<0>(coord)) + "," + std::to_string(std::get<1>(coord));
    };

    std::unordered_map<std::string, double> distances;
    std::unordered_map<std::string, Coord> parent;
    std::unordered_set<std::string> visited;

    auto cmp = [&distances, &coordToString](const Coord& a, const Coord& b) {
        return distances[coordToString(a)] > distances[coordToString(b)];
    };
    std::priority_queue<Coord, std::vector<Coord>, decltype(cmp)> pq(cmp);

    distances[coordToString(start)] = 0;
    pq.push(start);

    while (!pq.empty()) {
        auto current = pq.top();
        pq.pop();

        auto currentStr = coordToString(current);
        if (visited.find(currentStr) != visited.end()) {
            continue;
        }
        visited.insert(currentStr);

        if (current == end) {
            // Reconstruct path
            std::vector<Coord> path;
            auto node = end;
            while (node != start) {
                path.push_back(node);
                node = parent[coordToString(node)];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        auto neighbors = grid.getNeighbors(current);
        for (const auto& neighbor : neighbors) {
            auto neighborStr = coordToString(neighbor);
            if (visited.find(neighborStr) == visited.end()) {
                double newDist = distances[currentStr] + 1.0;
                if (distances.find(neighborStr) == distances.end() || newDist < distances[neighborStr]) {
                    distances[neighborStr] = newDist;
                    parent[neighborStr] = current;
                    pq.push(neighbor);
                }
            }
        }
    }

    return {};  // No path found
}

// Benchmark simple pathfinding on empty grid
static void PathfindingEmptyGrid_10x10(benchmark::State& state) {
    AStar_Grid<int> grid(10, 10);
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(9, 9);

    for (auto _ : state) {
        auto path = findPath(grid, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(PathfindingEmptyGrid_10x10);

static void PathfindingEmptyGrid_50x50(benchmark::State& state) {
    AStar_Grid<int> grid(50, 50);
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(49, 49);

    for (auto _ : state) {
        auto path = findPath(grid, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(PathfindingEmptyGrid_50x50);

static void PathfindingEmptyGrid_100x100(benchmark::State& state) {
    AStar_Grid<int> grid(100, 100);
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(99, 99);

    for (auto _ : state) {
        auto path = findPath(grid, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(PathfindingEmptyGrid_100x100);

static void PathfindingEmptyGrid_500x500(benchmark::State& state) {
    AStar_Grid<int> grid(500, 500);
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(499, 499);

    for (auto _ : state) {
        auto path = findPath(grid, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(PathfindingEmptyGrid_500x500);

// Benchmark with pre-generated verified obstacle maps
static void PathfindingWithObstacles_100x100_10Percent(benchmark::State& state) {
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(99, 99);

    for (auto _ : state) {
        auto path = findPath(grid_100x100_10pct, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(PathfindingWithObstacles_100x100_10Percent);

static void PathfindingWithObstacles_100x100_20Percent(benchmark::State& state) {
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(99, 99);

    for (auto _ : state) {
        auto path = findPath(grid_100x100_20pct, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(PathfindingWithObstacles_100x100_20Percent);

static void PathfindingWithObstacles_100x100_30Percent(benchmark::State& state) {
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(99, 99);

    for (auto _ : state) {
        auto path = findPath(grid_100x100_30pct, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(PathfindingWithObstacles_100x100_30Percent);

// Benchmark maze-like structures
static void PathfindingMaze_50x50(benchmark::State& state) {
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
BENCHMARK(PathfindingMaze_50x50);

// Benchmark worst case - no path exists
static void PathfindingNoPath_100x100(benchmark::State& state) {
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
BENCHMARK(PathfindingNoPath_100x100);

// Benchmark short paths
static void PathfindingShortPath_Adjacent(benchmark::State& state) {
    AStar_Grid<int> grid(10, 10);
    auto start = std::make_tuple(5, 5);
    auto end = std::make_tuple(6, 5);

    for (auto _ : state) {
        auto path = findPath(grid, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(PathfindingShortPath_Adjacent);

static void PathfindingShortPath_5Steps(benchmark::State& state) {
    AStar_Grid<int> grid(10, 10);
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(5, 0);

    for (auto _ : state) {
        auto path = findPath(grid, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(PathfindingShortPath_5Steps);

// Benchmark findPathSimple convenience function
static void FindPathSimple_100x100(benchmark::State& state) {
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
BENCHMARK(FindPathSimple_100x100);

// Benchmark Manhattan distance calculation
static void ManhattanDistance(benchmark::State& state) {
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(100, 100);

    for (auto _ : state) {
        auto distance = calculateManhattanDistance(start, end);
        benchmark::DoNotOptimize(distance);
    }
}
BENCHMARK(ManhattanDistance);

// Benchmark grid construction
static void GridConstruction_10x10(benchmark::State& state) {
    for (auto _ : state) {
        AStar_Grid<int> grid(10, 10);
        benchmark::DoNotOptimize(grid);
    }
}
BENCHMARK(GridConstruction_10x10);

static void GridConstruction_100x100(benchmark::State& state) {
    for (auto _ : state) {
        AStar_Grid<int> grid(100, 100);
        benchmark::DoNotOptimize(grid);
    }
}
BENCHMARK(GridConstruction_100x100);

static void GridConstruction_500x500(benchmark::State& state) {
    for (auto _ : state) {
        AStar_Grid<int> grid(500, 500);
        benchmark::DoNotOptimize(grid);
    }
}
BENCHMARK(GridConstruction_500x500);

// Benchmark neighbor retrieval
static void GetNeighbors(benchmark::State& state) {
    AStar_Grid<int> grid(100, 100);
    auto pos = std::make_tuple(50, 50);

    for (auto _ : state) {
        auto neighbors = grid.getNeighbors(pos);
        benchmark::DoNotOptimize(neighbors);
    }
}
BENCHMARK(GetNeighbors);

// ============================================================================
// Comparison Benchmarks: A* vs BFS vs Dijkstra
// ============================================================================

static void Comparison_AStar_50x50(benchmark::State& state) {
    AStar_Grid<int> grid(50, 50);
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(49, 49);

    for (auto _ : state) {
        auto path = findPath(grid, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(Comparison_AStar_50x50);

static void Comparison_BFS_50x50(benchmark::State& state) {
    AStar_Grid<int> grid(50, 50);
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(49, 49);

    for (auto _ : state) {
        auto path = bfs(grid, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(Comparison_BFS_50x50);

static void Comparison_Dijkstra_50x50(benchmark::State& state) {
    AStar_Grid<int> grid(50, 50);
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(49, 49);

    for (auto _ : state) {
        auto path = dijkstra(grid, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(Comparison_Dijkstra_50x50);

// With obstacles
static void Comparison_AStar_100x100_WithObstacles(benchmark::State& state) {
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(99, 99);

    for (auto _ : state) {
        auto path = findPath(grid_100x100_10pct, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(Comparison_AStar_100x100_WithObstacles);

static void Comparison_BFS_100x100_WithObstacles(benchmark::State& state) {
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(99, 99);

    for (auto _ : state) {
        auto path = bfs(grid_100x100_10pct, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(Comparison_BFS_100x100_WithObstacles);

static void Comparison_Dijkstra_100x100_WithObstacles(benchmark::State& state) {
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(99, 99);

    for (auto _ : state) {
        auto path = dijkstra(grid_100x100_10pct, start, end);
        benchmark::DoNotOptimize(path);
    }
}
BENCHMARK(Comparison_Dijkstra_100x100_WithObstacles);

// Custom main to initialize grids before running benchmarks
int main(int argc, char** argv) {
    initializeGrids();
    benchmark::Initialize(&argc, argv);
    if (benchmark::ReportUnrecognizedArguments(argc, argv)) return 1;
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
    return 0;
}
