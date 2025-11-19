#pragma once
#include <tuple>
#include <functional>

/**
 * @brief Common types and utilities for A* pathfinding
 */
namespace AStarLib
{
    /**
     * @brief Hash function for tuple coordinates
     * This is much more efficient than converting to string
     */
    template<typename CoordType>
    struct CoordHash
    {
        std::size_t operator()(const std::tuple<CoordType, CoordType>& coord) const noexcept
        {
            // Use a simple but effective hash combination
            std::size_t h1 = std::hash<CoordType>{}(std::get<0>(coord));
            std::size_t h2 = std::hash<CoordType>{}(std::get<1>(coord));
            // Cantor pairing function for combining hashes
            return h1 ^ (h2 << 1);
        }
    };

    /**
     * @brief Movement type for pathfinding
     */
    enum class MovementType
    {
        FourWay,    // Cardinal directions only (up, down, left, right)
        EightWay    // Include diagonals
    };
}
