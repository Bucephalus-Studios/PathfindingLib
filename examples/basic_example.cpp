/**
 * @file basic_example.cpp
 * @brief Basic example of using PathfindingLib for pathfinding
 */

#include "../PathfindingLib.hpp"
#include <iostream>
#include <iomanip>

using namespace PathfindingLib;

// Helper function to visualize the grid and path
void visualizeGrid(const Pathfinding_Grid<int>& grid,
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
    std::cout << "=== PathfindingLib Basic Example ===\n\n";

    // Example 1: Simple pathfinding
    {
        std::cout << "Example 1: Simple pathfinding on empty grid\n";

        Pathfinding_Grid<int> grid(10, 10);
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

        Pathfinding_Grid<int> grid(15, 10);

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

        Pathfinding_Grid<int> grid(10, 10);

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

        Pathfinding_Grid<int> grid(20, 20);
        auto start = std::make_tuple(0, 0);
        auto end = std::make_tuple(19, 19);

        // Add some obstacles
        for (int i = 5; i < 15; ++i) {
            grid.setObstacle(10, i);
        }

        auto path_manhattan = findPathAStar(grid, start, end, HeuristicType::Manhattan);
        auto path_euclidean = findPathAStar(grid, start, end, HeuristicType::Euclidean);

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

        Pathfinding_Grid<int> grid_4way(10, 10, MovementType::FourWay);
        Pathfinding_Grid<int> grid_8way(10, 10, MovementType::EightWay);

        auto start = std::make_tuple(0, 0);
        auto end = std::make_tuple(9, 9);

        auto path_4way = findPathAStar(grid_4way, start, end);
        auto path_8way = findPathAStar(grid_8way, start, end, HeuristicType::Octile);

        std::cout << "4-way movement path length: " << path_4way.size() << " steps\n";
        std::cout << "8-way movement path length: " << path_8way.size() << " steps\n";

        std::cout << "\n4-way path:\n";
        visualizeGrid(grid_4way, path_4way, start, end);

        std::cout << "8-way path (diagonal movement):\n";
        visualizeGrid(grid_8way, path_8way, start, end);
    }

    // Example 7: Comparing different algorithms
    {
        std::cout << "Example 7: Comparing different pathfinding algorithms\n";

        Pathfinding_Grid<int> grid(20, 20);

        // Add some obstacles
        for (int y = 5; y < 15; ++y) {
            grid.setObstacle(10, y);
        }

        auto start = std::make_tuple(5, 10);
        auto end = std::make_tuple(15, 10);

        auto pathAStar = findPathAStar(grid, start, end);
        auto pathDijkstra = findPathDijkstra(grid, start, end);
        auto pathBFS = findPathBFS(grid, start, end);
        auto pathDFS = findPathDFS(grid, start, end);
        auto pathGreedy = findPathGreedyBestFirst(grid, start, end);
        auto pathBidir = findPathBidirectionalAStar(grid, start, end);

        std::cout << "A* path length:                " << pathAStar.size() << " steps\n";
        std::cout << "Dijkstra path length:          " << pathDijkstra.size() << " steps\n";
        std::cout << "BFS path length:               " << pathBFS.size() << " steps\n";
        std::cout << "DFS path length:               " << pathDFS.size() << " steps (non-optimal)\n";
        std::cout << "Greedy Best-First path length: " << pathGreedy.size() << " steps (non-optimal)\n";
        std::cout << "Bidirectional A* path length:  " << pathBidir.size() << " steps\n\n";

        std::cout << "A* visualization:\n";
        visualizeGrid(grid, pathAStar, start, end);
    }

    // Example 8: Using the unified API with algorithm parameter
    {
        std::cout << "Example 8: Using unified findPath API\n";

        Pathfinding_Grid<int> grid(10, 10);
        auto start = std::make_tuple(0, 0);
        auto end = std::make_tuple(9, 9);

        auto pathAStar = findPath(grid, start, end, Algorithm::AStar);
        auto pathDijkstra = findPath(grid, start, end, Algorithm::Dijkstra);
        auto pathBFS = findPath(grid, start, end, Algorithm::BFS);

        std::cout << "Using Algorithm::AStar:    " << pathAStar.size() << " steps\n";
        std::cout << "Using Algorithm::Dijkstra: " << pathDijkstra.size() << " steps\n";
        std::cout << "Using Algorithm::BFS:      " << pathBFS.size() << " steps\n\n";
    }

    std::cout << "=== Examples Complete ===\n";

    return 0;
}
