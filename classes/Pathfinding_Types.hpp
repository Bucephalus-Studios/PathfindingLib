#pragma once
#include <tuple>
#include <functional>

/**
 * @brief Common types and utilities for pathfinding algorithms
 */
namespace PathfindingLib
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

    /**
     * @brief Pathfinding algorithm type
     */
    enum class Algorithm
    {
        AStar,              // A* - Optimal, balanced search using heuristic
        Dijkstra,           // Dijkstra - Optimal, guaranteed shortest path (no heuristic)
        BFS,                // Breadth-First Search - Optimal for unweighted graphs
        DFS,                // Depth-First Search - Fast but non-optimal
        GreedyBestFirst,    // Greedy Best-First - Fast but non-optimal, heuristic only
        BidirectionalAStar  // Bidirectional A* - Searches from both ends
    };

    /**
     * @brief Heuristic function type
     */
    enum class HeuristicType
    {
        Manhattan,  // L1 distance (best for 4-way movement)
        Euclidean,  // L2 distance (best for 8-way movement)
        Chebyshev,  // L-infinity distance (alternative for 8-way)
        Octile      // Optimal for 8-way movement with different diagonal cost
    };
}
