# PathfindingLib

A fast, header-only C++17 pathfinding library with **6 algorithms**, extensive testing, and comprehensive benchmarking.

## Features

- ✨ Header-only library (easy integration)
- 🎯 **6 Pathfinding Algorithms**: A*, Dijkstra, BFS, DFS, Greedy Best-First, Bidirectional A*
- 🚀 **60% faster** than naive implementations (optimized hash functions)
- 🧪 Comprehensive test coverage (90+ unit tests)
- 📊 Performance benchmarks for all algorithms
- 🎨 Multiple heuristic functions (Manhattan, Euclidean, Chebyshev, Octile)
- 🔄 Support for 4-way and 8-way movement
- 📐 Template-based coordinate types (int, float, double, etc.)
- 🎓 Well-documented with examples

## Quick Start

### Installation

This is a header-only library. Simply copy the header files to your project:

```cpp
#include "PathfindingLib.hpp"
```

### Basic Example

```cpp
#include "PathfindingLib.hpp"
#include <iostream>

using namespace PathfindingLib;

int main() {
    // Create a 10x10 grid
    Pathfinding_Grid<int> grid(10, 10);

    // Add some obstacles
    grid.setObstacle(5, 5);
    grid.setObstacle(5, 6);
    grid.setObstacle(5, 7);

    // Find path from (0,0) to (9,9) using A*
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(9, 9);
    auto path = findPathAStar(grid, start, end);

    // Print the path
    if (!path.empty()) {
        std::cout << "Path found with " << path.size() << " steps:\n";
        for (const auto& [x, y] : path) {
            std::cout << "(" << x << "," << y << ") ";
        }
        std::cout << "\n";
    } else {
        std::cout << "No path found!\n";
    }

    return 0;
}
```

## Algorithms

PathfindingLib supports **6 different pathfinding algorithms**, each with different characteristics:

| Algorithm            | Optimal | Use Case                                    |
|---------------------|---------|---------------------------------------------|
| **A***              | ✅ Yes  | Best general-purpose algorithm              |
| **Dijkstra**        | ✅ Yes  | Guaranteed shortest path, no heuristic      |
| **BFS**             | ✅ Yes  | Simple, optimal for unweighted graphs       |
| **DFS**             | ❌ No   | Fast, non-optimal, explores deeply          |
| **Greedy Best-First** | ❌ No | Very fast, follows heuristic greedily       |
| **Bidirectional A*** | ✅ Yes | Searches from both ends, faster for long paths |

### Using Different Algorithms

#### Option 1: Direct Function Calls

```cpp
using namespace PathfindingLib;
Pathfinding_Grid<int> grid(20, 20);
auto start = std::make_tuple(0, 0);
auto end = std::make_tuple(19, 19);

// A* - Optimal, balanced search
auto pathAStar = findPathAStar(grid, start, end);

// Dijkstra - Optimal, guaranteed shortest
auto pathDijkstra = findPathDijkstra(grid, start, end);

// BFS - Optimal for unweighted graphs
auto pathBFS = findPathBFS(grid, start, end);

// DFS - Fast but non-optimal
auto pathDFS = findPathDFS(grid, start, end);

// Greedy Best-First - Very fast, heuristic-driven
auto pathGreedy = findPathGreedyBestFirst(grid, start, end);

// Bidirectional A* - Searches from both ends
auto pathBidirectional = findPathBidirectionalAStar(grid, start, end);
```

#### Option 2: Unified API

```cpp
using namespace PathfindingLib;
Pathfinding_Grid<int> grid(20, 20);

// Use the unified findPath() function with algorithm parameter
auto pathAStar = findPath(grid, start, end, Algorithm::AStar);
auto pathDijkstra = findPath(grid, start, end, Algorithm::Dijkstra);
auto pathBFS = findPath(grid, start, end, Algorithm::BFS);
auto pathDFS = findPath(grid, start, end, Algorithm::DFS);
auto pathGreedy = findPath(grid, start, end, Algorithm::GreedyBestFirst);
auto pathBidir = findPath(grid, start, end, Algorithm::BidirectionalAStar);
```

### Simple Convenience Function

For quick pathfinding without creating a grid manually:

```cpp
#include "PathfindingLib.hpp"

int main() {
    std::vector<std::tuple<int, int>> obstacles = {
        {5, 5}, {5, 6}, {5, 7}
    };

    // Use any algorithm with findPathSimple
    auto path = PathfindingLib::findPathSimple(
        10, 10,                          // Grid size
        obstacles,                        // Obstacle list
        std::make_tuple(0, 0),           // Start
        std::make_tuple(9, 9),           // End
        Algorithm::AStar                 // Algorithm choice
    );

    return 0;
}
```

## Advanced Features

### Different Heuristics

Choose the best heuristic for your use case (works with A*, Greedy Best-First, and Bidirectional A*):

```cpp
using namespace PathfindingLib;
Pathfinding_Grid<int> grid(20, 20);

// Manhattan distance (best for 4-way movement)
auto path1 = findPathAStar(grid, start, end, HeuristicType::Manhattan);

// Euclidean distance (best for 8-way movement)
auto path2 = findPathAStar(grid, start, end, HeuristicType::Euclidean);

// Chebyshev distance (L-infinity norm)
auto path3 = findPathAStar(grid, start, end, HeuristicType::Chebyshev);

// Octile distance (optimal for 8-way with diagonal cost)
auto path4 = findPathAStar(grid, start, end, HeuristicType::Octile);
```

### 8-Way Movement

Enable diagonal movement:

```cpp
// Create grid with 8-way movement
Pathfinding_Grid<int> grid(20, 20, MovementType::EightWay);

// Use Octile heuristic for best results with 8-way movement
auto path = findPathAStar(grid, start, end, HeuristicType::Octile);
```

### Custom Coordinate Types

Use floating-point coordinates for continuous spaces:

```cpp
Pathfinding_Grid<float> grid(100.0f, 100.0f);
auto start = std::make_tuple(0.0f, 0.0f);
auto end = std::make_tuple(99.5f, 99.5f);
auto path = findPathAStar(grid, start, end);
```

## Building Tests and Benchmarks

```bash
mkdir build && cd build
cmake ..
cmake --build .

# Run tests (90+ tests covering all algorithms)
ctest --output-on-failure

# Run benchmarks (compares all algorithms)
./benchmarks/benchmarks
```

## Performance

Benchmarks show excellent performance across all algorithms:

### A* Performance
| Grid Size | Time (ns)  | Operations/sec |
|-----------|------------|----------------|
| 10x10     | ~3,400     | ~294,000       |
| 50x50     | ~20,700    | ~48,300        |
| 100x100   | ~49,900    | ~20,000        |
| 500x500   | ~277,000   | ~3,600         |

**60% faster** than naive string-based coordinate hashing!

### Algorithm Comparison (50x50 grid, empty)
| Algorithm         | Relative Speed | Optimality |
|-------------------|----------------|------------|
| Greedy Best-First | Fastest        | ❌         |
| DFS               | Very Fast      | ❌         |
| A*                | Fast           | ✅         |
| BFS               | Fast           | ✅         |
| Dijkstra          | Moderate       | ✅         |
| Bidirectional A*  | Fast           | ✅         |

*Note: Performance varies based on grid size, obstacle density, and path length.*

## API Reference

### Pathfinding Functions

#### Algorithm-Specific Functions

```cpp
// A* Algorithm
template<typename CoordType>
std::vector<std::tuple<CoordType, CoordType>> findPathAStar(
    Pathfinding_Grid<CoordType>& grid,
    const std::tuple<CoordType, CoordType>& start,
    const std::tuple<CoordType, CoordType>& end,
    HeuristicType heuristic = HeuristicType::Manhattan
);

// Dijkstra Algorithm
template<typename CoordType>
std::vector<std::tuple<CoordType, CoordType>> findPathDijkstra(
    Pathfinding_Grid<CoordType>& grid,
    const std::tuple<CoordType, CoordType>& start,
    const std::tuple<CoordType, CoordType>& end
);

// Breadth-First Search
template<typename CoordType>
std::vector<std::tuple<CoordType, CoordType>> findPathBFS(
    Pathfinding_Grid<CoordType>& grid,
    const std::tuple<CoordType, CoordType>& start,
    const std::tuple<CoordType, CoordType>& end
);

// Depth-First Search
template<typename CoordType>
std::vector<std::tuple<CoordType, CoordType>> findPathDFS(
    Pathfinding_Grid<CoordType>& grid,
    const std::tuple<CoordType, CoordType>& start,
    const std::tuple<CoordType, CoordType>& end
);

// Greedy Best-First Search
template<typename CoordType>
std::vector<std::tuple<CoordType, CoordType>> findPathGreedyBestFirst(
    Pathfinding_Grid<CoordType>& grid,
    const std::tuple<CoordType, CoordType>& start,
    const std::tuple<CoordType, CoordType>& end,
    HeuristicType heuristic = HeuristicType::Manhattan
);

// Bidirectional A*
template<typename CoordType>
std::vector<std::tuple<CoordType, CoordType>> findPathBidirectionalAStar(
    Pathfinding_Grid<CoordType>& grid,
    const std::tuple<CoordType, CoordType>& start,
    const std::tuple<CoordType, CoordType>& end,
    HeuristicType heuristic = HeuristicType::Manhattan
);
```

#### Unified API

```cpp
template<typename CoordType>
std::vector<std::tuple<CoordType, CoordType>> findPath(
    Pathfinding_Grid<CoordType>& grid,
    const std::tuple<CoordType, CoordType>& start,
    const std::tuple<CoordType, CoordType>& end,
    Algorithm algorithm = Algorithm::AStar,
    HeuristicType heuristic = HeuristicType::Manhattan
);
```

#### Convenience Function

```cpp
template<typename CoordType>
std::vector<std::tuple<CoordType, CoordType>> findPathSimple(
    CoordType width,
    CoordType height,
    const std::vector<std::tuple<CoordType, CoordType>>& obstacles,
    const std::tuple<CoordType, CoordType>& start,
    const std::tuple<CoordType, CoordType>& end,
    Algorithm algorithm = Algorithm::AStar,
    HeuristicType heuristic = HeuristicType::Manhattan
);
```

### Grid Class

#### Constructor
```cpp
Pathfinding_Grid(
    CoordType width,
    CoordType height,
    MovementType moveType = MovementType::FourWay
);
```

#### Methods
- `setObstacle(x, y)` - Mark a cell as unwalkable
- `setWalkable(x, y)` - Mark a cell as walkable
- `getNode(x, y)` - Get node at position
- `isWithinBounds(x, y)` - Check if coordinates are valid
- `getNeighbors(x, y)` - Get neighboring cells (4-way or 8-way)
- `reset()` - Clear all pathfinding state
- `getMovementType()` / `setMovementType()` - Get/set movement type

### Enums

```cpp
enum class Algorithm {
    AStar,              // A* algorithm
    Dijkstra,           // Dijkstra's algorithm
    BFS,                // Breadth-First Search
    DFS,                // Depth-First Search
    GreedyBestFirst,    // Greedy Best-First Search
    BidirectionalAStar  // Bidirectional A*
};

enum class HeuristicType {
    Manhattan,  // L1 norm (best for 4-way)
    Euclidean,  // L2 norm (best for 8-way)
    Chebyshev,  // L-infinity norm
    Octile      // Optimal for 8-way with diagonal cost
};

enum class MovementType {
    FourWay,    // Cardinal directions only
    EightWay    // Include diagonals
};
```

### Heuristic Functions

- `calculateManhattanDistance()` - L1 norm (best for 4-way)
- `calculateEuclideanDistance()` - L2 norm (best for 8-way)
- `calculateChebyshevDistance()` - L-infinity norm
- `calculateOctileDistance()` - Optimal for 8-way with diagonal cost

## Requirements

- C++17 or later
- CMake 3.14+ (for building tests/benchmarks)
- Google Test (automatically fetched)
- Google Benchmark (automatically fetched)

## Examples

See the `examples/` directory for:
- Basic pathfinding
- Pathfinding with obstacles
- Different heuristics
- 8-way movement
- **Algorithm comparison**
- **Unified API usage**

## License

See LICENSE file for details.

## Contributing

Contributions are welcome! Please ensure:
- All tests pass
- Code follows the existing style
- New features include tests
- Performance-critical code includes benchmarks

## Changelog

### Version 3.0.0 (Breaking Changes)
- ⚠️ **BREAKING**: Removed old AStarLib backward compatibility files
- 🧹 Cleaned up legacy code for cleaner codebase
- 📝 All code now uses PathfindingLib namespace and types

### Version 2.0.0
- 🎯 Added 5 new pathfinding algorithms (Dijkstra, BFS, DFS, Greedy Best-First, Bidirectional A*)
- 🎨 Introduced unified `findPath()` API with algorithm parameter
- 📝 Renamed library to PathfindingLib to reflect broader capabilities
- 🧪 Expanded test suite to 90+ tests
- 📊 Added comprehensive benchmarks for all algorithms
- 🔙 Maintained full backward compatibility with AStarLib 1.x

### Version 1.0.0
- Initial release as AStarLib
- A* pathfinding implementation
- 4 heuristic functions
- 52 unit tests
- Performance benchmarks
