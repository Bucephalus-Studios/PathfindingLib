#include "../AStar_Node.hpp"
#include <gtest/gtest.h>

// Test basic node construction
TEST(AStarNodeTest, DefaultConstruction) {
    AStar_Node<int> node;
    EXPECT_EQ(std::get<0>(node.position), 0);
    EXPECT_EQ(std::get<1>(node.position), 0);
    EXPECT_EQ(node.gCost, 0);
    EXPECT_EQ(node.hCost, 0);
    EXPECT_EQ(node.fCost, 0);
    EXPECT_TRUE(node.isWalkable);
    EXPECT_EQ(std::get<0>(node.parent), -1);
    EXPECT_EQ(std::get<1>(node.parent), -1);
}

TEST(AStarNodeTest, ConstructionWithTuple) {
    auto pos = std::make_tuple(5, 10);
    AStar_Node<int> node(pos);
    EXPECT_EQ(node.position, pos);
    EXPECT_TRUE(node.isWalkable);
}

TEST(AStarNodeTest, ConstructionWithCoordinates) {
    AStar_Node<int> node(7, 3);
    EXPECT_EQ(std::get<0>(node.position), 7);
    EXPECT_EQ(std::get<1>(node.position), 3);
    EXPECT_TRUE(node.isWalkable);
}

TEST(AStarNodeTest, ConstructionWithWalkability) {
    AStar_Node<int> walkableNode(1, 2, true);
    AStar_Node<int> unwalkableNode(3, 4, false);

    EXPECT_TRUE(walkableNode.isWalkable);
    EXPECT_FALSE(unwalkableNode.isWalkable);
}

// Test cost calculations
TEST(AStarNodeTest, CalculateFCost) {
    AStar_Node<int> node;
    node.gCost = 5;
    node.hCost = 10;
    node.calculateFCost();
    EXPECT_EQ(node.fCost, 15);
}

TEST(AStarNodeTest, GetPosition) {
    AStar_Node<int> node(8, 12);
    auto pos = node.getPosition();
    EXPECT_EQ(std::get<0>(pos), 8);
    EXPECT_EQ(std::get<1>(pos), 12);
}

// Test walkability
TEST(AStarNodeTest, WalkabilityGettersAndSetters) {
    AStar_Node<int> node;
    EXPECT_TRUE(node.getIsWalkable());

    node.setWalkable(false);
    EXPECT_FALSE(node.getIsWalkable());

    node.setWalkable(true);
    EXPECT_TRUE(node.getIsWalkable());
}

// Test with different coordinate types
TEST(AStarNodeTest, FloatCoordinates) {
    AStar_Node<float> node(3.5f, 7.2f);
    EXPECT_FLOAT_EQ(std::get<0>(node.position), 3.5f);
    EXPECT_FLOAT_EQ(std::get<1>(node.position), 7.2f);
}

TEST(AStarNodeTest, DoubleCoordinates) {
    AStar_Node<double> node(3.14159, 2.71828);
    EXPECT_DOUBLE_EQ(std::get<0>(node.position), 3.14159);
    EXPECT_DOUBLE_EQ(std::get<1>(node.position), 2.71828);
}

// Test parent tracking
TEST(AStarNodeTest, ParentTracking) {
    AStar_Node<int> node(5, 5);
    EXPECT_EQ(std::get<0>(node.parent), -1);
    EXPECT_EQ(std::get<1>(node.parent), -1);

    node.parent = std::make_tuple(4, 5);
    EXPECT_EQ(std::get<0>(node.parent), 4);
    EXPECT_EQ(std::get<1>(node.parent), 5);
}
