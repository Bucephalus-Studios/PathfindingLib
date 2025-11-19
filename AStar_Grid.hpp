#pragma once
#include "AStar_Node.hpp"
#include "AStar_Types.hpp"
#include <vector>

/**
 * @brief Represents a 2D grid for A* pathfinding
 */
template<typename CoordType>
class AStar_Grid
{
public:
    // Public data members for backward compatibility
    // Note: Direct access maintained for compatibility with existing code
    std::vector<std::vector<AStar_Node<CoordType>>> nodes;
    CoordType width;
    CoordType height;

private:
    AStarLib::MovementType movementType_;

public:
    /**
     * @brief Constructor
     * @param w Grid width
     * @param h Grid height
     * @param moveType Movement type (FourWay or EightWay)
     */
    AStar_Grid(CoordType w, CoordType h, AStarLib::MovementType moveType = AStarLib::MovementType::FourWay)
        : nodes(), width(w), height(h), movementType_(moveType)
    {
        // Initialize the grid with walkable nodes
        nodes.resize(width);
        for (CoordType x = 0; x < width; ++x)
        {
            nodes[x].resize(height);
            for (CoordType y = 0; y < height; ++y)
            {
                nodes[x][y] = AStar_Node<CoordType>(x, y, true);
            }
        }
    }

    /**
     * @brief Get grid width
     */
    CoordType getWidth() const noexcept { return width; }

    /**
     * @brief Get grid height
     */
    CoordType getHeight() const noexcept { return height; }

    /**
     * @brief Get movement type
     */
    AStarLib::MovementType getMovementType() const noexcept { return movementType_; }

    /**
     * @brief Set movement type
     */
    void setMovementType(AStarLib::MovementType moveType) noexcept { movementType_ = moveType; }
    
    /**
     * @brief Get a node at the specified position
     */
    AStar_Node<CoordType>& getNode(CoordType x, CoordType y)
    {
        return nodes[x][y];
    }

    /**
     * @brief Get a node at the specified position (const version)
     */
    const AStar_Node<CoordType>& getNode(CoordType x, CoordType y) const
    {
        return nodes[x][y];
    }

    /**
     * @brief Get a node at the specified position using tuple
     */
    AStar_Node<CoordType>& getNode(const std::tuple<CoordType, CoordType>& pos)
    {
        return nodes[std::get<0>(pos)][std::get<1>(pos)];
    }

    /**
     * @brief Get a node at the specified position using tuple (const version)
     */
    const AStar_Node<CoordType>& getNode(const std::tuple<CoordType, CoordType>& pos) const
    {
        return nodes[std::get<0>(pos)][std::get<1>(pos)];
    }
    
    /**
     * @brief Check if coordinates are within the grid bounds
     */
    bool isWithinBounds(CoordType x, CoordType y) const noexcept
    {
        return x >= 0 && x < width && y >= 0 && y < height;
    }
    
    /**
     * @brief Check if coordinates are within the grid bounds using tuple
     */
    bool isWithinBounds(const std::tuple<CoordType, CoordType>& pos) const noexcept
    {
        return isWithinBounds(std::get<0>(pos), std::get<1>(pos));
    }
    
    /**
     * @brief Set a node as unwalkable (obstacle)
     */
    void setObstacle(CoordType x, CoordType y)
    {
        if (isWithinBounds(x, y))
        {
            nodes[x][y].setWalkable(false);
        }
    }

    /**
     * @brief Set a node as walkable
     */
    void setWalkable(CoordType x, CoordType y)
    {
        if (isWithinBounds(x, y))
        {
            nodes[x][y].setWalkable(true);
        }
    }
    
    /**
     * @brief Get neighboring nodes based on movement type
     * @param x X coordinate
     * @param y Y coordinate
     * @return Vector of neighboring coordinates
     */
    std::vector<std::tuple<CoordType, CoordType>> getNeighbors(CoordType x, CoordType y) const
    {
        std::vector<std::tuple<CoordType, CoordType>> neighbors;
        neighbors.reserve(movementType_ == AStarLib::MovementType::EightWay ? 8 : 4);

        // Cardinal directions (always included)
        static constexpr int cardinalDirs[4][2] = {
            {0, 1},   // Up
            {1, 0},   // Right
            {0, -1},  // Down
            {-1, 0}   // Left
        };

        for (const auto& [dx, dy] : cardinalDirs)
        {
            CoordType newX = x + dx;
            CoordType newY = y + dy;

            if (isWithinBounds(newX, newY))
            {
                neighbors.emplace_back(newX, newY);
            }
        }

        // Diagonal directions (only if 8-way movement)
        if (movementType_ == AStarLib::MovementType::EightWay)
        {
            static constexpr int diagonalDirs[4][2] = {
                {1, 1},   // Up-Right
                {1, -1},  // Down-Right
                {-1, -1}, // Down-Left
                {-1, 1}   // Up-Left
            };

            for (const auto& [dx, dy] : diagonalDirs)
            {
                CoordType newX = x + dx;
                CoordType newY = y + dy;

                if (isWithinBounds(newX, newY))
                {
                    neighbors.emplace_back(newX, newY);
                }
            }
        }

        return neighbors;
    }
    
    /**
     * @brief Get neighboring nodes using tuple position
     */
    std::vector<std::tuple<CoordType, CoordType>> getNeighbors(const std::tuple<CoordType, CoordType>& pos) const
    {
        return getNeighbors(std::get<0>(pos), std::get<1>(pos));
    }

    /**
     * @brief Reset all nodes (clear costs and parents)
     */
    void reset()
    {
        for (CoordType x = 0; x < width; ++x)
        {
            for (CoordType y = 0; y < height; ++y)
            {
                auto& node = nodes[x][y];
                node.gCost = 0;
                node.hCost = 0;
                node.fCost = 0;
                node.parent = std::make_tuple(-1, -1);
            }
        }
    }
};