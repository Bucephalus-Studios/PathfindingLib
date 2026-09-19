#pragma once
#include "Pathfinding_Node.hpp"
#include "Pathfinding_Types.hpp"
#include <functional>
#include <vector>

/**
 * @brief Represents a 2D grid for pathfinding algorithms
 */
template<typename CoordType>
class Pathfinding_Grid
{
public:
    // Public data members for backward compatibility
    // Note: Direct access maintained for compatibility with existing code
    std::vector<std::vector<Pathfinding_Node<CoordType>>> nodes;
    CoordType width;
    CoordType height;

private:
    PathfindingLib::MovementType movementType;
    std::function<CoordType(CoordType, CoordType)> costFunction;

public:
    /**
     * @brief Constructor
     * @param w Grid width
     * @param h Grid height
     * @param moveType Movement type (FourWay or EightWay)
     */
    Pathfinding_Grid(CoordType w, CoordType h, PathfindingLib::MovementType moveType = PathfindingLib::MovementType::FourWay)
        : nodes(), width(w), height(h), movementType(moveType)
    {
        // Initialize the grid with walkable nodes
        nodes.resize(width);
        for (CoordType x = 0; x < width; ++x)
        {
            nodes[x].resize(height);
            for (CoordType y = 0; y < height; ++y)
            {
                nodes[x][y] = Pathfinding_Node<CoordType>(x, y, true);
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
    PathfindingLib::MovementType getMovementType() const noexcept { return movementType; }

    /**
     * @brief Set movement type
     */
    void setMovementType(PathfindingLib::MovementType moveType) noexcept { movementType = moveType; }

    /**
     * @brief Get a node at the specified position
     */
    Pathfinding_Node<CoordType>& getNode(CoordType x, CoordType y)
    {
        return nodes[x][y];
    }

    /**
     * @brief Get a node at the specified position (const version)
     */
    const Pathfinding_Node<CoordType>& getNode(CoordType x, CoordType y) const
    {
        return nodes[x][y];
    }

    /**
     * @brief Get a node at the specified position using tuple
     */
    Pathfinding_Node<CoordType>& getNode(const std::tuple<CoordType, CoordType>& pos)
    {
        return nodes[std::get<0>(pos)][std::get<1>(pos)];
    }

    /**
     * @brief Get a node at the specified position using tuple (const version)
     */
    const Pathfinding_Node<CoordType>& getNode(const std::tuple<CoordType, CoordType>& pos) const
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
     * @brief Set the cost to move into a node (default 1 -- see Pathfinding_Node::cost).
     */
    void setCost(CoordType x, CoordType y, CoordType cost)
    {
        if (isWithinBounds(x, y))
        {
            nodes[x][y].cost = cost;
        }
    }

    /**
     * @brief Populates every node's cost immediately via a per-tile cost function fn(x, y) ->
     *        cost. For terrain-weighted grids where cost varies per tile (elevation, movement
     *        speed, fuel, etc.) -- the caller supplies the domain-specific formula, this just
     *        handles applying it across the grid up front. Use setLazyGridTravelCosts()
     *        instead when fn is expensive and the grid might not need every tile's cost this
     *        call (e.g. a flood fill capped by findCostField()'s maxCost parameter).
     */
    template<typename CostFunction>
    void setGridTravelCosts(CostFunction fn)
    {
        costFunction = fn;
        for (CoordType x = 0; x < width; ++x)
        {
            for (CoordType y = 0; y < height; ++y)
            {
                nodes[x][y].cost = costFunction(x, y);
                nodes[x][y].costComputed = true;
            }
        }
    }

    /**
     * @brief Stores a per-tile cost function fn(x, y) -> cost WITHOUT evaluating it -- each
     *        node's cost is computed (and cached) on first read via getCost(), not up front.
     *        Use this when fn is expensive (e.g. depends on faction/character effect lookups)
     *        and the grid may only ever need a fraction of its tiles' costs this call, such as
     *        a flood fill capped by findCostField()'s maxCost parameter.
     */
    template<typename CostFunction>
    void setLazyGridTravelCosts(CostFunction fn)
    {
        costFunction = fn;
        for (CoordType x = 0; x < width; ++x)
        {
            for (CoordType y = 0; y < height; ++y)
            {
                nodes[x][y].costComputed = false;
            }
        }
    }

    /**
     * @brief Reads a node's cost, computing and caching it first if this grid is in lazy mode
     *        (setLazyGridTravelCosts()) and this tile hasn't been visited yet. Eager grids
     *        (setGridTravelCosts(), or the flat default cost) always have costComputed already
     *        true, so this is just a plain read for them -- safe to call unconditionally
     *        either way, which is why findCostField() goes through this instead of touching
     *        node.cost directly.
     */
    CoordType getCost(CoordType x, CoordType y)
    {
        Pathfinding_Node<CoordType> & node = nodes[x][y];
        if (!node.costComputed)
        {
            node.cost = costFunction(x, y);
            node.costComputed = true;
        }
        return node.cost;
    }

    /**
     * @brief Tuple-coordinate overload of getCost(x, y).
     */
    CoordType getCost(const std::tuple<CoordType, CoordType> & pos)
    {
        return getCost(std::get<0>(pos), std::get<1>(pos));
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
        neighbors.reserve(movementType == PathfindingLib::MovementType::EightWay ? 8 : 4);

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
        if (movementType == PathfindingLib::MovementType::EightWay)
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
