#pragma once
#include <tuple>

/**
 * @brief Represents a node in pathfinding algorithms
 */
template<typename CoordType>
struct Pathfinding_Node
{
    std::tuple<CoordType, CoordType> position;
    CoordType gCost;  // Distance from start node
    CoordType hCost;  // Heuristic distance to end node
    CoordType fCost;  // Total cost (g + h)
    CoordType cost = CoordType(1);   // Cost to move INTO this node; findPathAStar() adds this
                                      // instead of a flat 1 per step, so weighted terrain is
                                      // opt-in -- the default reproduces old unweighted behavior.
    bool costComputed = true;        // false only under Pathfinding_Grid's lazy cost mode (see
                                      // setLazyGridTravelCosts()) -- means `cost` above hasn't
                                      // been evaluated yet for this node. Grids that never opt
                                      // into lazy costs (setGridTravelCosts(), or the flat
                                      // default above) leave every node already "computed", so
                                      // Pathfinding_Grid::getCost() works the same either way.
    std::tuple<CoordType, CoordType> parent; // Parent node position for path reconstruction
    bool isWalkable;

    Pathfinding_Node()
        : position(std::make_tuple(0, 0)), gCost(0), hCost(0), fCost(0),
          parent(std::make_tuple(-1, -1)), isWalkable(true) {}

    Pathfinding_Node(const std::tuple<CoordType, CoordType>& pos, bool walkable = true)
        : position(pos), gCost(0), hCost(0), fCost(0),
          parent(std::make_tuple(-1, -1)), isWalkable(walkable) {}

    Pathfinding_Node(CoordType x, CoordType y, bool walkable = true)
        : position(std::make_tuple(x, y)), gCost(0), hCost(0), fCost(0),
          parent(std::make_tuple(-1, -1)), isWalkable(walkable) {}

    /**
     * @brief Calculate and update the F cost (G + H)
     */
    void calculateFCost()
    {
        fCost = gCost + hCost;
    }

    /**
     * @brief Get the position of this node
     */
    std::tuple<CoordType, CoordType> getPosition() const
    {
        return position;
    }

    /**
     * @brief Check if this node is walkable
     */
    bool getIsWalkable() const
    {
        return isWalkable;
    }

    /**
     * @brief Set walkability of this node
     */
    void setWalkable(bool walkable)
    {
        isWalkable = walkable;
    }
};
