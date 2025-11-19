#include "../AStarLib.hpp"
#include <gtest/gtest.h>
#include <random>

using namespace AStarLib;

// Test Manhattan distance calculation
TEST(PathfindingTest, ManhattanDistanceHorizontal) {
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(5, 0);
    EXPECT_EQ(calculateManhattanDistance(start, end), 5);
}

TEST(PathfindingTest, ManhattanDistanceVertical) {
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(0, 10);
    EXPECT_EQ(calculateManhattanDistance(start, end), 10);
}

TEST(PathfindingTest, ManhattanDistanceDiagonal) {
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(3, 4);
    EXPECT_EQ(calculateManhattanDistance(start, end), 7);
}

TEST(PathfindingTest, ManhattanDistanceNegativeCoords) {
    auto start = std::make_tuple(-5, -3);
    auto end = std::make_tuple(2, 4);
    EXPECT_EQ(calculateManhattanDistance(start, end), 14);
}

TEST(PathfindingTest, ManhattanDistanceSamePoint) {
    auto start = std::make_tuple(5, 5);
    auto end = std::make_tuple(5, 5);
    EXPECT_EQ(calculateManhattanDistance(start, end), 0);
}

// Test coordToString helper (deprecated but testing for backward compatibility)
TEST(PathfindingTest, CoordToString) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    auto coord = std::make_tuple(10, 20);
    std::string result = coordToString(coord);
    EXPECT_EQ(result, "10,20");
#pragma GCC diagnostic pop
}

// Test basic pathfinding
TEST(PathfindingTest, SimpleStraightPath) {
    AStar_Grid<int> grid(10, 10);
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(5, 0);

    auto path = findPath(grid, start, end);

    EXPECT_FALSE(path.empty());
    EXPECT_EQ(path.front(), start);
    EXPECT_EQ(path.back(), end);
    EXPECT_EQ(path.size(), 6);  // 0,0 -> 1,0 -> 2,0 -> 3,0 -> 4,0 -> 5,0
}

TEST(PathfindingTest, PathAroundObstacle) {
    AStar_Grid<int> grid(5, 5);

    // Create a wall
    grid.setObstacle(2, 0);
    grid.setObstacle(2, 1);
    grid.setObstacle(2, 2);

    auto start = std::make_tuple(0, 1);
    auto end = std::make_tuple(4, 1);

    auto path = findPath(grid, start, end);

    EXPECT_FALSE(path.empty());
    EXPECT_EQ(path.front(), start);
    EXPECT_EQ(path.back(), end);

    // Path should go around the wall, not through it
    for (const auto& coord : path) {
        EXPECT_TRUE(grid.getNode(coord).getIsWalkable());
    }
}

TEST(PathfindingTest, NoPathDueToObstacles) {
    AStar_Grid<int> grid(5, 5);

    // Create a complete wall
    for (int y = 0; y < 5; ++y) {
        grid.setObstacle(2, y);
    }

    auto start = std::make_tuple(0, 2);
    auto end = std::make_tuple(4, 2);

    auto path = findPath(grid, start, end);

    EXPECT_TRUE(path.empty());  // No path should exist
}

TEST(PathfindingTest, StartEqualsEnd) {
    AStar_Grid<int> grid(10, 10);
    auto pos = std::make_tuple(5, 5);

    auto path = findPath(grid, pos, pos);

    EXPECT_EQ(path.size(), 1);
    EXPECT_EQ(path[0], pos);
}

TEST(PathfindingTest, StartOutOfBounds) {
    AStar_Grid<int> grid(10, 10);
    auto start = std::make_tuple(-1, 5);
    auto end = std::make_tuple(5, 5);

    auto path = findPath(grid, start, end);

    EXPECT_TRUE(path.empty());
}

TEST(PathfindingTest, EndOutOfBounds) {
    AStar_Grid<int> grid(10, 10);
    auto start = std::make_tuple(5, 5);
    auto end = std::make_tuple(15, 5);

    auto path = findPath(grid, start, end);

    EXPECT_TRUE(path.empty());
}

TEST(PathfindingTest, StartIsObstacle) {
    AStar_Grid<int> grid(10, 10);
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(5, 5);

    grid.setObstacle(0, 0);

    auto path = findPath(grid, start, end);

    EXPECT_TRUE(path.empty());
}

TEST(PathfindingTest, EndIsObstacle) {
    AStar_Grid<int> grid(10, 10);
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(5, 5);

    grid.setObstacle(5, 5);

    auto path = findPath(grid, start, end);

    EXPECT_TRUE(path.empty());
}

TEST(PathfindingTest, PathConnectivity) {
    AStar_Grid<int> grid(10, 10);
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(9, 9);

    auto path = findPath(grid, start, end);

    EXPECT_FALSE(path.empty());

    // Check that each step in path is adjacent to the next
    for (size_t i = 0; i < path.size() - 1; ++i) {
        int x1 = std::get<0>(path[i]);
        int y1 = std::get<1>(path[i]);
        int x2 = std::get<0>(path[i + 1]);
        int y2 = std::get<1>(path[i + 1]);

        int dx = std::abs(x2 - x1);
        int dy = std::abs(y2 - y1);

        // Should move exactly one step in one direction
        EXPECT_TRUE((dx == 1 && dy == 0) || (dx == 0 && dy == 1));
    }
}

TEST(PathfindingTest, ComplexMaze) {
    AStar_Grid<int> grid(10, 10);

    // Create a complex maze pattern
    for (int y = 1; y < 9; y += 2) {
        for (int x = 1; x < 9; x += 2) {
            grid.setObstacle(x, y);
        }
    }

    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(9, 9);

    auto path = findPath(grid, start, end);

    EXPECT_FALSE(path.empty());
    EXPECT_EQ(path.front(), start);
    EXPECT_EQ(path.back(), end);
}

// Test findPathSimple convenience function
TEST(PathfindingTest, FindPathSimpleNoObstacles) {
    std::vector<std::tuple<int, int>> obstacles;
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(5, 5);

    auto path = findPathSimple(10, 10, obstacles, start, end);

    EXPECT_FALSE(path.empty());
    EXPECT_EQ(path.front(), start);
    EXPECT_EQ(path.back(), end);
}

TEST(PathfindingTest, FindPathSimpleWithObstacles) {
    std::vector<std::tuple<int, int>> obstacles = {
        {2, 0}, {2, 1}, {2, 2}, {2, 3}
    };
    auto start = std::make_tuple(0, 2);
    auto end = std::make_tuple(4, 2);

    auto path = findPathSimple(10, 10, obstacles, start, end);

    EXPECT_FALSE(path.empty());
    EXPECT_EQ(path.front(), start);
    EXPECT_EQ(path.back(), end);

    // Verify path doesn't go through obstacles
    for (const auto& coord : path) {
        EXPECT_EQ(std::find(obstacles.begin(), obstacles.end(), coord), obstacles.end());
    }
}

TEST(PathfindingTest, FindPathSimpleNoPath) {
    std::vector<std::tuple<int, int>> obstacles;
    // Create a complete vertical wall
    for (int y = 0; y < 10; ++y) {
        obstacles.push_back({5, y});
    }

    auto start = std::make_tuple(0, 5);
    auto end = std::make_tuple(9, 5);

    auto path = findPathSimple(10, 10, obstacles, start, end);

    EXPECT_TRUE(path.empty());
}

// Test optimal path selection
TEST(PathfindingTest, OptimalPathSelection) {
    AStar_Grid<int> grid(10, 10);
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(5, 0);

    auto path = findPath(grid, start, end);

    // Should take the shortest path (straight line)
    EXPECT_EQ(path.size(), 6);

    // Verify all y-coordinates are 0 (straight horizontal path)
    for (const auto& coord : path) {
        EXPECT_EQ(std::get<1>(coord), 0);
    }
}

// Test path length calculations
TEST(PathfindingTest, PathLengthMatchesManhattanDistance) {
    AStar_Grid<int> grid(20, 20);
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(10, 10);

    auto path = findPath(grid, start, end);

    EXPECT_FALSE(path.empty());
    // Path length should be Manhattan distance + 1 (includes both start and end)
    EXPECT_EQ(path.size(), calculateManhattanDistance(start, end) + 1);
}

// Test large grids
TEST(PathfindingTest, LargeGrid) {
    AStar_Grid<int> grid(100, 100);
    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(99, 99);

    auto path = findPath(grid, start, end);

    EXPECT_FALSE(path.empty());
    EXPECT_EQ(path.front(), start);
    EXPECT_EQ(path.back(), end);
}

// Test edge cases with adjacent points
TEST(PathfindingTest, AdjacentPointsHorizontal) {
    AStar_Grid<int> grid(10, 10);
    auto start = std::make_tuple(5, 5);
    auto end = std::make_tuple(6, 5);

    auto path = findPath(grid, start, end);

    EXPECT_EQ(path.size(), 2);
    EXPECT_EQ(path[0], start);
    EXPECT_EQ(path[1], end);
}

TEST(PathfindingTest, AdjacentPointsVertical) {
    AStar_Grid<int> grid(10, 10);
    auto start = std::make_tuple(5, 5);
    auto end = std::make_tuple(5, 6);

    auto path = findPath(grid, start, end);

    EXPECT_EQ(path.size(), 2);
    EXPECT_EQ(path[0], start);
    EXPECT_EQ(path[1], end);
}

// Test 20% obstacles scenario to verify benchmark correctness
TEST(PathfindingTest, PathfindingWith20PercentObstacles) {
    std::mt19937 rng(12345);  // Same seed as benchmark
    std::uniform_int_distribution<int> dist(0, 99);

    AStar_Grid<int> grid(100, 100);

    // Add 20% obstacles (same as benchmark)
    int obstacleCount = 2000;
    for (int i = 0; i < obstacleCount; ++i) {
        int x = dist(rng);
        int y = dist(rng);
        grid.setObstacle(x, y);
    }

    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(99, 99);

    auto path = findPath(grid, start, end);

    // Verify behavior - path might exist or not depending on obstacle placement
    // But this should not be instantaneous
    if (!path.empty()) {
        EXPECT_EQ(path.front(), start);
        EXPECT_EQ(path.back(), end);
        // Verify all nodes in path are walkable
        for (const auto& coord : path) {
            EXPECT_TRUE(grid.getNode(coord).getIsWalkable());
        }
    }
}

// Test 10% obstacles scenario for comparison
TEST(PathfindingTest, PathfindingWith10PercentObstacles) {
    std::mt19937 rng(12345);
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

    auto path = findPath(grid, start, end);

    EXPECT_FALSE(path.empty());
    EXPECT_EQ(path.front(), start);
    EXPECT_EQ(path.back(), end);
}

// Test to verify that start/end are not blocked by random obstacles
TEST(PathfindingTest, RandomObstaclesDoNotBlockStartEnd) {
    std::mt19937 rng(54321);
    std::uniform_int_distribution<int> dist(0, 99);

    AStar_Grid<int> grid(100, 100);

    // Add random obstacles
    for (int i = 0; i < 1500; ++i) {
        int x = dist(rng);
        int y = dist(rng);
        // Don't block start or end
        if ((x != 0 || y != 0) && (x != 99 || y != 99)) {
            grid.setObstacle(x, y);
        }
    }

    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(99, 99);

    // Verify start and end are walkable
    EXPECT_TRUE(grid.getNode(start).getIsWalkable());
    EXPECT_TRUE(grid.getNode(end).getIsWalkable());

    auto path = findPath(grid, start, end);

    // Path should exist since start and end are guaranteed walkable
    // (though may still fail if completely walled off)
    if (path.empty()) {
        // If no path, verify that at least one endpoint neighbor is blocked
        bool startHasWalkableNeighbor = false;
        auto startNeighbors = grid.getNeighbors(start);
        for (const auto& n : startNeighbors) {
            if (grid.getNode(n).getIsWalkable()) {
                startHasWalkableNeighbor = true;
                break;
            }
        }
        // Only acceptable if start is completely surrounded
        EXPECT_FALSE(startHasWalkableNeighbor);
    }
}
