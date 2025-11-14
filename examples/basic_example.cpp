/**
 * @file basic_example.cpp
 * @brief Basic example of using AStarLib for pathfinding
 */

#include "../AStarLib.hpp"
#include <iostream>
#include <iomanip>

using namespace AStarLib;

// Helper function to visualize the grid and path
void visualizeGrid(const AStar_Grid<int>& grid,
                   const std::vector<std::tuple<int, int>>& path,
                   const std::tuple<int, int>& start,
                   const std::tuple<int, int>& end)
{
    std::cout << "\nGrid Visualization:\n";
    std::cout << "S = Start, E = End, * = Path, # = Obstacle, . = Empty\n\n";

    for (int y = grid.height - 1; y >= 0; --y)
    {
        for (int x = 0; x < grid.width; ++x)
        {
            auto pos = std::make_tuple(x, y);

            if (pos == start) {
                std::cout << "S ";
            } else if (pos == end) {
                std::cout << "E ";
            } else if (std::find(path.begin(), path.end(), pos) != path.end()) {
                std::cout << "* ";
            } else if (!grid.getNode(x, y).getIsWalkable()) {
                std::cout << "# ";
            } else {
                std::cout << ". ";
            }
        }
        std::cout << "\n";
    }
    std::cout << "\n";
}

int main()
{
    std::cout << "=== AStarLib Basic Example ===\n\n";

    // Example 1: Simple pathfinding
    {
        std::cout << "Example 1: Simple pathfinding on empty grid\n";

        AStar_Grid<int> grid(10, 10);
        auto start = std::make_tuple(0, 0);
        auto end = std::make_tuple(9, 9);

        auto path = findPath(grid, start, end);

        std::cout << "Path length: " << path.size() << " steps\n";
        std::cout << "Path: ";
        for (const auto& [x, y] : path) {
            std::cout << "(" << x << "," << y << ") ";
        }
        std::cout << "\n";

        visualizeGrid(grid, path, start, end);
    }

    // Example 2: Pathfinding around obstacles
    {
        std::cout << "Example 2: Pathfinding around obstacles\n";

        AStar_Grid<int> grid(15, 10);

        // Create a vertical wall
        for (int y = 2; y < 8; ++y) {
            grid.setObstacle(7, y);
        }

        auto start = std::make_tuple(2, 5);
        auto end = std::make_tuple(12, 5);

        auto path = findPath(grid, start, end);

        std::cout << "Path length: " << path.size() << " steps\n";

        visualizeGrid(grid, path, start, end);
    }

    // Example 3: No path available
    {
        std::cout << "Example 3: No path available (completely blocked)\n";

        AStar_Grid<int> grid(10, 10);

        // Create impassable wall
        for (int y = 0; y < 10; ++y) {
            grid.setObstacle(5, y);
        }

        auto start = std::make_tuple(2, 5);
        auto end = std::make_tuple(8, 5);

        auto path = findPath(grid, start, end);

        if (path.empty()) {
            std::cout << "No path found (as expected)!\n\n";
        }
    }

    // Example 4: Using different heuristics
    {
        std::cout << "Example 4: Comparing different heuristics\n";

        AStar_Grid<int> grid(20, 20);
        auto start = std::make_tuple(0, 0);
        auto end = std::make_tuple(19, 19);

        // Add some obstacles
        for (int i = 5; i < 15; ++i) {
            grid.setObstacle(10, i);
        }

        auto path_manhattan = findPath(grid, start, end, HeuristicType::Manhattan);
        auto path_euclidean = findPath(grid, start, end, HeuristicType::Euclidean);

        std::cout << "Manhattan heuristic path length: " << path_manhattan.size() << "\n";
        std::cout << "Euclidean heuristic path length: " << path_euclidean.size() << "\n\n";
    }

    // Example 5: Using the convenience function
    {
        std::cout << "Example 5: Using findPathSimple()\n";

        std::vector<std::tuple<int, int>> obstacles = {
            {5, 5}, {5, 6}, {5, 7}, {5, 8}
        };

        auto path = findPathSimple(
            10, 10,
            obstacles,
            std::make_tuple(0, 5),
            std::make_tuple(9, 5)
        );

        std::cout << "Path length: " << path.size() << " steps\n\n";
    }

    // Example 6: 8-way movement
    {
        std::cout << "Example 6: 8-way movement (diagonal allowed)\n";

        AStar_Grid<int> grid_4way(10, 10, MovementType::FourWay);
        AStar_Grid<int> grid_8way(10, 10, MovementType::EightWay);

        auto start = std::make_tuple(0, 0);
        auto end = std::make_tuple(9, 9);

        auto path_4way = findPath(grid_4way, start, end);
        auto path_8way = findPath(grid_8way, start, end, HeuristicType::Octile);

        std::cout << "4-way movement path length: " << path_4way.size() << " steps\n";
        std::cout << "8-way movement path length: " << path_8way.size() << " steps\n";

        std::cout << "\n4-way path:\n";
        visualizeGrid(grid_4way, path_4way, start, end);

        std::cout << "8-way path (diagonal movement):\n";
        visualizeGrid(grid_8way, path_8way, start, end);
    }

    std::cout << "=== Examples Complete ===\n";

    return 0;
}
