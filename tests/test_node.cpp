#include "../Pathfinding_Node.hpp"
#include <gtest/gtest.h>

// Test basic node construction
TEST(PathfindingNodeTest, DefaultConstruction) {
    Pathfinding_Node<int> node;
    EXPECT_EQ(std::get<0>(node.position), 0);
    EXPECT_EQ(std::get<1>(node.position), 0);
    EXPECT_EQ(node.gCost, 0);
    EXPECT_EQ(node.hCost, 0);
    EXPECT_EQ(node.fCost, 0);
    EXPECT_TRUE(node.isWalkable);
    EXPECT_EQ(std::get<0>(node.parent), -1);
    EXPECT_EQ(std::get<1>(node.parent), -1);
}

TEST(PathfindingNodeTest, ConstructionWithTuple) {
    auto pos = std::make_tuple(5, 10);
    Pathfinding_Node<int> node(pos);
    EXPECT_EQ(node.position, pos);
    EXPECT_TRUE(node.isWalkable);
}

TEST(PathfindingNodeTest, ConstructionWithCoordinates) {
    Pathfinding_Node<int> node(7, 3);
    EXPECT_EQ(std::get<0>(node.position), 7);
    EXPECT_EQ(std::get<1>(node.position), 3);
    EXPECT_TRUE(node.isWalkable);
}

TEST(PathfindingNodeTest, ConstructionWithWalkability) {
    Pathfinding_Node<int> walkableNode(1, 2, true);
    Pathfinding_Node<int> unwalkableNode(3, 4, false);

    EXPECT_TRUE(walkableNode.isWalkable);
    EXPECT_FALSE(unwalkableNode.isWalkable);
}

// Test cost calculations
TEST(PathfindingNodeTest, CalculateFCost) {
    Pathfinding_Node<int> node;
    node.gCost = 5;
    node.hCost = 10;
    node.calculateFCost();
    EXPECT_EQ(node.fCost, 15);
}

TEST(PathfindingNodeTest, GetPosition) {
    Pathfinding_Node<int> node(8, 12);
    auto pos = node.getPosition();
    EXPECT_EQ(std::get<0>(pos), 8);
    EXPECT_EQ(std::get<1>(pos), 12);
}

// Test walkability
TEST(PathfindingNodeTest, WalkabilityGettersAndSetters) {
    Pathfinding_Node<int> node;
    EXPECT_TRUE(node.getIsWalkable());

    node.setWalkable(false);
    EXPECT_FALSE(node.getIsWalkable());

    node.setWalkable(true);
    EXPECT_TRUE(node.getIsWalkable());
}

// Test with different coordinate types
TEST(PathfindingNodeTest, FloatCoordinates) {
    Pathfinding_Node<float> node(3.5f, 7.2f);
    EXPECT_FLOAT_EQ(std::get<0>(node.position), 3.5f);
    EXPECT_FLOAT_EQ(std::get<1>(node.position), 7.2f);
}

TEST(PathfindingNodeTest, DoubleCoordinates) {
    Pathfinding_Node<double> node(3.14159, 2.71828);
    EXPECT_DOUBLE_EQ(std::get<0>(node.position), 3.14159);
    EXPECT_DOUBLE_EQ(std::get<1>(node.position), 2.71828);
}

// Test parent tracking
TEST(PathfindingNodeTest, ParentTracking) {
    Pathfinding_Node<int> node(5, 5);
    EXPECT_EQ(std::get<0>(node.parent), -1);
    EXPECT_EQ(std::get<1>(node.parent), -1);

    node.parent = std::make_tuple(4, 5);
    EXPECT_EQ(std::get<0>(node.parent), 4);
    EXPECT_EQ(std::get<1>(node.parent), 5);
}
