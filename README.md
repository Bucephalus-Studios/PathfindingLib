# AStarLib

A fast, header-only C++17 A* pathfinding library with extensive testing and benchmarking.

## Features

- ✨ Header-only library (easy integration)
- 🚀 **60% faster** than naive implementations (optimized hash functions)
- 🧪 Comprehensive test coverage (52 unit tests)
- 📊 Performance benchmarks included
- 🎯 Multiple heuristic functions (Manhattan, Euclidean, Chebyshev, Octile)
- 🔄 Support for 4-way and 8-way movement
- 📐 Template-based coordinate types (int, float, double, etc.)
- 🎓 Well-documented with examples

## Quick Start

### Installation

This is a header-only library. Simply copy the header files to your project:

```cpp
#include "AStarLib.hpp"
```

### Basic Example

```cpp
#include "AStarLib.hpp"
#include <iostream>

using namespace AStarLib;

int main() {
    // Create a 10x10 grid
    AStar_Grid<int> grid(10, 10);

    // Add some obstacles
    grid.setObstacle(5, 5);
    grid.setObstacle(5, 6);
    grid.setObstacle(5, 7);

    // Find path from (0,0) to (9,9)
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(9, 9);
    auto path = findPath(grid, start, end);

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

### Simple Convenience Function

For quick pathfinding without creating a grid manually:

```cpp
#include "AStarLib.hpp"

int main() {
    std::vector<std::tuple<int, int>> obstacles = {
        {5, 5}, {5, 6}, {5, 7}
    };

    auto path = AStarLib::findPathSimple(
        10, 10,                          // Grid size
        obstacles,                        // Obstacle list
        std::make_tuple(0, 0),           // Start
        std::make_tuple(9, 9)            // End
    );

    return 0;
}
```

## Advanced Features

### Different Heuristics

Choose the best heuristic for your use case:

```cpp
using namespace AStarLib;

AStar_Grid<int> grid(20, 20);

// Manhattan distance (best for 4-way movement)
auto path1 = findPath(grid, start, end, HeuristicType::Manhattan);

// Euclidean distance (best for 8-way movement)
auto path2 = findPath(grid, start, end, HeuristicType::Euclidean);

// Chebyshev distance (L-infinity norm)
auto path3 = findPath(grid, start, end, HeuristicType::Chebyshev);

// Octile distance (optimal for 8-way with diagonal cost)
auto path4 = findPath(grid, start, end, HeuristicType::Octile);
```

### 8-Way Movement

Enable diagonal movement:

```cpp
// Create grid with 8-way movement
AStar_Grid<int> grid(20, 20, MovementType::EightWay);

auto path = findPath(grid, start, end, HeuristicType::Octile);
```

### Custom Coordinate Types

Use floating-point coordinates for continuous spaces:

```cpp
AStar_Grid<float> grid(100.0f, 100.0f);
auto start = std::make_tuple(0.0f, 0.0f);
auto end = std::make_tuple(99.5f, 99.5f);
auto path = findPath(grid, start, end);
```

## Building Tests and Benchmarks

```bash
mkdir build && cd build
cmake ..
cmake --build .

# Run tests
ctest --output-on-failure

# Run benchmarks
./benchmarks/astar_benchmarks
```

## Performance

Benchmarks on a typical system show:

| Grid Size | Time (ns)  | Operations/sec |
|-----------|------------|----------------|
| 10x10     | 3,437      | ~291,000       |
| 50x50     | 20,726     | ~48,000        |
| 100x100   | 49,936     | ~20,000        |
| 500x500   | 277,354    | ~3,600         |

**60% faster** than naive string-based coordinate hashing!

## API Reference

### Core Functions

#### `findPath()`
```cpp
template<typename CoordType>
std::vector<std::tuple<CoordType, CoordType>> findPath(
    AStar_Grid<CoordType>& grid,
    const std::tuple<CoordType, CoordType>& start,
    const std::tuple<CoordType, CoordType>& end,
    HeuristicType heuristic = HeuristicType::Manhattan
);
```

Finds the shortest path from `start` to `end` using the A* algorithm.

**Parameters:**
- `grid`: The grid to search on
- `start`: Starting position (x, y)
- `end`: Target position (x, y)
- `heuristic`: Heuristic function to use (optional)

**Returns:** Vector of coordinates representing the path (empty if no path found)

#### `findPathSimple()`
```cpp
template<typename CoordType>
std::vector<std::tuple<CoordType, CoordType>> findPathSimple(
    CoordType width,
    CoordType height,
    const std::vector<std::tuple<CoordType, CoordType>>& obstacles,
    const std::tuple<CoordType, CoordType>& start,
    const std::tuple<CoordType, CoordType>& end
);
```

Convenience function that creates a grid and finds a path in one call.

### Grid Class

#### Constructor
```cpp
AStar_Grid(
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
- `getNeighbors(x, y)` - Get neighboring cells
- `reset()` - Clear all pathfinding state

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

## License

See LICENSE file for details.

## Contributing

Contributions are welcome! Please ensure:
- All tests pass
- Code follows the existing style
- New features include tests
- Performance-critical code includes benchmarks
