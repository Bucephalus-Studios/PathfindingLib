#include "../classes/Pathfinding_Grid.hpp"
#include <gtest/gtest.h>

// Test grid construction
TEST(PathfindingGridTest, Construction) {
    Pathfinding_Grid<int> grid(10, 20);
    EXPECT_EQ(grid.width, 10);
    EXPECT_EQ(grid.height, 20);
    EXPECT_EQ(grid.nodes.size(), 10);
    EXPECT_EQ(grid.nodes[0].size(), 20);
}

TEST(PathfindingGridTest, AllNodesInitializedAsWalkable) {
    Pathfinding_Grid<int> grid(5, 5);
    for (int x = 0; x < 5; ++x) {
        for (int y = 0; y < 5; ++y) {
            EXPECT_TRUE(grid.getNode(x, y).getIsWalkable());
        }
    }
}

// Test node access
TEST(PathfindingGridTest, GetNodeWithCoordinates) {
    Pathfinding_Grid<int> grid(10, 10);
    auto& node = grid.getNode(3, 7);
    EXPECT_EQ(std::get<0>(node.position), 3);
    EXPECT_EQ(std::get<1>(node.position), 7);
}

TEST(PathfindingGridTest, GetNodeWithTuple) {
    Pathfinding_Grid<int> grid(10, 10);
    auto pos = std::make_tuple(5, 2);
    auto& node = grid.getNode(pos);
    EXPECT_EQ(node.position, pos);
}

TEST(PathfindingGridTest, GetNodeConst) {
    const Pathfinding_Grid<int> grid(10, 10);
    const auto& node = grid.getNode(4, 6);
    EXPECT_EQ(std::get<0>(node.position), 4);
    EXPECT_EQ(std::get<1>(node.position), 6);
}

// Test bounds checking
TEST(PathfindingGridTest, IsWithinBoundsValid) {
    Pathfinding_Grid<int> grid(10, 15);
    EXPECT_TRUE(grid.isWithinBounds(0, 0));
    EXPECT_TRUE(grid.isWithinBounds(9, 14));
    EXPECT_TRUE(grid.isWithinBounds(5, 7));
}

TEST(PathfindingGridTest, IsWithinBoundsInvalid) {
    Pathfinding_Grid<int> grid(10, 15);
    EXPECT_FALSE(grid.isWithinBounds(-1, 5));
    EXPECT_FALSE(grid.isWithinBounds(5, -1));
    EXPECT_FALSE(grid.isWithinBounds(10, 5));
    EXPECT_FALSE(grid.isWithinBounds(5, 15));
    EXPECT_FALSE(grid.isWithinBounds(-1, -1));
    EXPECT_FALSE(grid.isWithinBounds(20, 20));
}

TEST(PathfindingGridTest, IsWithinBoundsWithTuple) {
    Pathfinding_Grid<int> grid(10, 10);
    EXPECT_TRUE(grid.isWithinBounds(std::make_tuple(5, 5)));
    EXPECT_FALSE(grid.isWithinBounds(std::make_tuple(-1, 5)));
    EXPECT_FALSE(grid.isWithinBounds(std::make_tuple(10, 5)));
}

// Test obstacle setting
TEST(PathfindingGridTest, SetObstacle) {
    Pathfinding_Grid<int> grid(10, 10);
    EXPECT_TRUE(grid.getNode(5, 5).getIsWalkable());

    grid.setObstacle(5, 5);
    EXPECT_FALSE(grid.getNode(5, 5).getIsWalkable());
}

TEST(PathfindingGridTest, SetObstacleOutOfBounds) {
    Pathfinding_Grid<int> grid(10, 10);
    // Should not crash when setting obstacle out of bounds
    grid.setObstacle(-1, 5);
    grid.setObstacle(15, 5);
    grid.setObstacle(5, -1);
    grid.setObstacle(5, 20);
}

TEST(PathfindingGridTest, SetWalkable) {
    Pathfinding_Grid<int> grid(10, 10);
    grid.setObstacle(3, 3);
    EXPECT_FALSE(grid.getNode(3, 3).getIsWalkable());

    grid.setWalkable(3, 3);
    EXPECT_TRUE(grid.getNode(3, 3).getIsWalkable());
}

// Test neighbor finding
TEST(PathfindingGridTest, GetNeighborsCenter) {
    Pathfinding_Grid<int> grid(10, 10);
    auto neighbors = grid.getNeighbors(5, 5);

    EXPECT_EQ(neighbors.size(), 4);

    // Check that we get all 4 cardinal directions
    bool hasUp = false, hasDown = false, hasLeft = false, hasRight = false;
    for (const auto& neighbor : neighbors) {
        if (neighbor == std::make_tuple(5, 6)) hasUp = true;
        if (neighbor == std::make_tuple(5, 4)) hasDown = true;
        if (neighbor == std::make_tuple(4, 5)) hasLeft = true;
        if (neighbor == std::make_tuple(6, 5)) hasRight = true;
    }

    EXPECT_TRUE(hasUp);
    EXPECT_TRUE(hasDown);
    EXPECT_TRUE(hasLeft);
    EXPECT_TRUE(hasRight);
}

TEST(PathfindingGridTest, GetNeighborsCorner) {
    Pathfinding_Grid<int> grid(10, 10);
    auto neighbors = grid.getNeighbors(0, 0);

    EXPECT_EQ(neighbors.size(), 2);  // Only right and up
}

TEST(PathfindingGridTest, GetNeighborsEdge) {
    Pathfinding_Grid<int> grid(10, 10);
    auto neighbors = grid.getNeighbors(0, 5);

    EXPECT_EQ(neighbors.size(), 3);  // Up, down, and right
}

TEST(PathfindingGridTest, GetNeighborsWithTuple) {
    Pathfinding_Grid<int> grid(10, 10);
    auto pos = std::make_tuple(5, 5);
    auto neighbors = grid.getNeighbors(pos);

    EXPECT_EQ(neighbors.size(), 4);
}

// Test with different coordinate types
TEST(PathfindingGridTest, UnsignedIntCoordinates) {
    Pathfinding_Grid<unsigned int> grid(10, 10);
    EXPECT_EQ(grid.width, 10u);
    EXPECT_EQ(grid.height, 10u);
}

// Test grid sizes
TEST(PathfindingGridTest, SmallGrid) {
    Pathfinding_Grid<int> grid(1, 1);
    EXPECT_EQ(grid.width, 1);
    EXPECT_EQ(grid.height, 1);
    auto neighbors = grid.getNeighbors(0, 0);
    EXPECT_EQ(neighbors.size(), 0);  // Single cell has no neighbors
}

TEST(PathfindingGridTest, LargeGrid) {
    Pathfinding_Grid<int> grid(100, 100);
    EXPECT_EQ(grid.width, 100);
    EXPECT_EQ(grid.height, 100);
    EXPECT_TRUE(grid.isWithinBounds(99, 99));
    EXPECT_FALSE(grid.isWithinBounds(100, 100));
}
