#pragma once
#include "AStar_Node.hpp"
#include "AStar_Grid.hpp"
#include "AStar_Types.hpp"
#include <vector>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <string>

/**
 * @brief A library for A* pathfinding algorithm
 */
namespace AStarLib
{
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

    /**
     * @brief Calculate Manhattan distance heuristic (L1 norm)
     * Best for 4-way movement
     */
    template<typename CoordType>
    CoordType calculateManhattanDistance(const std::tuple<CoordType, CoordType>& start,
                                        const std::tuple<CoordType, CoordType>& end) noexcept
    {
        CoordType dx = std::abs(std::get<0>(end) - std::get<0>(start));
        CoordType dy = std::abs(std::get<1>(end) - std::get<1>(start));
        return dx + dy;
    }

    /**
     * @brief Calculate Euclidean distance heuristic (L2 norm)
     * Best for 8-way movement with accurate distances
     */
    template<typename CoordType>
    CoordType calculateEuclideanDistance(const std::tuple<CoordType, CoordType>& start,
                                        const std::tuple<CoordType, CoordType>& end) noexcept
    {
        CoordType dx = std::get<0>(end) - std::get<0>(start);
        CoordType dy = std::get<1>(end) - std::get<1>(start);
        return static_cast<CoordType>(std::sqrt(dx * dx + dy * dy));
    }

    /**
     * @brief Calculate Chebyshev distance heuristic (L-infinity norm)
     * Best for 8-way movement where diagonal and straight moves cost the same
     */
    template<typename CoordType>
    CoordType calculateChebyshevDistance(const std::tuple<CoordType, CoordType>& start,
                                         const std::tuple<CoordType, CoordType>& end) noexcept
    {
        CoordType dx = std::abs(std::get<0>(end) - std::get<0>(start));
        CoordType dy = std::abs(std::get<1>(end) - std::get<1>(start));
        return std::max(dx, dy);
    }

    /**
     * @brief Calculate Octile distance heuristic
     * Optimal for 8-way movement where diagonal moves cost sqrt(2)
     */
    template<typename CoordType>
    CoordType calculateOctileDistance(const std::tuple<CoordType, CoordType>& start,
                                      const std::tuple<CoordType, CoordType>& end) noexcept
    {
        CoordType dx = std::abs(std::get<0>(end) - std::get<0>(start));
        CoordType dy = std::abs(std::get<1>(end) - std::get<1>(start));
        CoordType dMin = std::min(dx, dy);
        CoordType dMax = std::max(dx, dy);
        return dMin * static_cast<CoordType>(1.414213562) + (dMax - dMin);
    }

    /**
     * @brief Helper function to convert coordinate tuple to string for hashing
     * @deprecated Use CoordHash struct instead for better performance
     */
    template<typename CoordType>
    [[deprecated("Use CoordHash for better performance")]]
    std::string coordToString(const std::tuple<CoordType, CoordType>& coord)
    {
        return std::to_string(std::get<0>(coord)) + "," + std::to_string(std::get<1>(coord));
    }
    

    /**
     * @brief Comparator for priority queue (min-heap based on fCost)
     */
    template<typename CoordType>
    struct NodeComparator
    {
        const AStar_Grid<CoordType>* grid;

        explicit NodeComparator(const AStar_Grid<CoordType>* g) : grid(g) {}

        bool operator()(const std::tuple<CoordType, CoordType>& a,
                       const std::tuple<CoordType, CoordType>& b) const
        {
            const auto& nodeA = grid->getNode(a);
            const auto& nodeB = grid->getNode(b);

            // If F costs are equal, prefer lower H cost (closer to target)
            if (nodeA.fCost == nodeB.fCost)
            {
                return nodeA.hCost > nodeB.hCost; // Min-heap, so reverse comparison
            }
            return nodeA.fCost > nodeB.fCost; // Min-heap, so reverse comparison
        }
    };
    
    
    /**
     * @brief Find path using A* algorithm with selectable heuristic
     *
     * @param grid The grid to search on
     * @param start Starting position
     * @param end Target position
     * @param heuristic Heuristic function to use
     * @return Vector of coordinates representing the path (empty if no path found)
     */
    template<typename CoordType>
    std::vector<std::tuple<CoordType, CoordType>> findPath(
        AStar_Grid<CoordType>& grid,
        const std::tuple<CoordType, CoordType>& start,
        const std::tuple<CoordType, CoordType>& end,
        HeuristicType heuristic = HeuristicType::Manhattan)
    {
        std::vector<std::tuple<CoordType, CoordType>> path;
        path.reserve(32); // Reserve space to reduce allocations

        // Check if start and end are valid and walkable
        if (!grid.isWithinBounds(start) || !grid.isWithinBounds(end))
        {
            return path; // Empty path
        }

        if (!grid.getNode(start).getIsWalkable() || !grid.getNode(end).getIsWalkable())
        {
            return path; // Empty path
        }

        // If start and end are the same
        if (start == end)
        {
            path.push_back(start);
            return path;
        }

        // Select heuristic function
        auto heuristicFunc = [heuristic](const auto& a, const auto& b) -> CoordType {
            switch (heuristic)
            {
                case HeuristicType::Euclidean:
                    return calculateEuclideanDistance(a, b);
                case HeuristicType::Chebyshev:
                    return calculateChebyshevDistance(a, b);
                case HeuristicType::Octile:
                    return calculateOctileDistance(a, b);
                case HeuristicType::Manhattan:
                default:
                    return calculateManhattanDistance(a, b);
            }
        };

        // Initialize start node
        auto& startNode = grid.getNode(start);
        startNode.gCost = 0;
        startNode.hCost = heuristicFunc(start, end);
        startNode.calculateFCost();
        startNode.parent = std::make_tuple(-1, -1); // No parent

        // Priority queue for open set (nodes to be evaluated)
        std::priority_queue<std::tuple<CoordType, CoordType>,
                           std::vector<std::tuple<CoordType, CoordType>>,
                           NodeComparator<CoordType>> openSet{NodeComparator<CoordType>(&grid)};

        // Use optimized hash function instead of string conversion
        std::unordered_set<std::tuple<CoordType, CoordType>, CoordHash<CoordType>> openSetTracker;
        std::unordered_set<std::tuple<CoordType, CoordType>, CoordHash<CoordType>> closedSet;

        openSet.push(start);
        openSetTracker.insert(start);

        while (!openSet.empty())
        {
            // Get node with lowest fCost
            auto current = openSet.top();
            openSet.pop();

            openSetTracker.erase(current);
            closedSet.insert(current);

            // Check if we reached the target
            if (current == end)
            {
                // Reconstruct path
                path.clear();
                auto pathNode = current;
                while (!(std::get<0>(grid.getNode(pathNode).parent) == -1 &&
                        std::get<1>(grid.getNode(pathNode).parent) == -1))
                {
                    path.push_back(pathNode);
                    pathNode = grid.getNode(pathNode).parent;
                }
                path.push_back(start);

                // Reverse to get path from start to end
                std::reverse(path.begin(), path.end());
                return path;
            }

            // Check all neighbors
            const auto neighbors = grid.getNeighbors(current);
            for (const auto& neighbor : neighbors)
            {
                // Skip if neighbor is not walkable or already in closed set
                if (!grid.getNode(neighbor).getIsWalkable() ||
                    closedSet.find(neighbor) != closedSet.end())
                {
                    continue;
                }

                // Calculate tentative gCost (cost of 1 for each step)
                CoordType tentativeGCost = grid.getNode(current).gCost + 1;

                auto& neighborNode = grid.getNode(neighbor);

                // If this path to neighbor is better than any previous one
                if (openSetTracker.find(neighbor) == openSetTracker.end() ||
                    tentativeGCost < neighborNode.gCost)
                {
                    // This path is the best until now
                    neighborNode.parent = current;
                    neighborNode.gCost = tentativeGCost;
                    neighborNode.hCost = heuristicFunc(neighbor, end);
                    neighborNode.calculateFCost();

                    // Add to open set if not already there
                    if (openSetTracker.find(neighbor) == openSetTracker.end())
                    {
                        openSet.push(neighbor);
                        openSetTracker.insert(neighbor);
                    }
                }
            }
        }

        // No path found
        return path; // Empty path
    }
    

    /**
     * @brief Convenience function to find path between two points on a simple grid
     * 
     * @param width Grid width
     * @param height Grid height
     * @param obstacles Set of obstacle coordinates
     * @param start Starting position
     * @param end Target position
     * @return Vector of coordinates representing the path
     */
    template<typename CoordType>
    std::vector<std::tuple<CoordType, CoordType>> findPathSimple(
        CoordType width, CoordType height,
        const std::vector<std::tuple<CoordType, CoordType>>& obstacles,
        const std::tuple<CoordType, CoordType>& start,
        const std::tuple<CoordType, CoordType>& end)
    {
        AStar_Grid<CoordType> grid(width, height);
        
        // Set obstacles
        for (const auto& obstacle : obstacles) 
        {
            grid.setObstacle(std::get<0>(obstacle), std::get<1>(obstacle));
        }
        
        return findPath(grid, start, end);
    }
}