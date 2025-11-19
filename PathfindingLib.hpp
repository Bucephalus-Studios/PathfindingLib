#pragma once
#include "classes/Pathfinding_Node.hpp"
#include "classes/Pathfinding_Grid.hpp"
#include "classes/Pathfinding_Types.hpp"
#include <vector>
#include <queue>
#include <stack>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <string>

/**
 * @brief A comprehensive library for pathfinding algorithms
 */
namespace PathfindingLib
{
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
        const Pathfinding_Grid<CoordType>* grid;

        explicit NodeComparator(const Pathfinding_Grid<CoordType>* g) : grid(g) {}

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
     * @brief Comparator for Greedy Best-First (only uses hCost)
     */
    template<typename CoordType>
    struct GreedyComparator
    {
        const Pathfinding_Grid<CoordType>* grid;

        explicit GreedyComparator(const Pathfinding_Grid<CoordType>* g) : grid(g) {}

        bool operator()(const std::tuple<CoordType, CoordType>& a,
                       const std::tuple<CoordType, CoordType>& b) const
        {
            const auto& nodeA = grid->getNode(a);
            const auto& nodeB = grid->getNode(b);
            return nodeA.hCost > nodeB.hCost; // Min-heap, so reverse comparison
        }
    };

    /**
     * @brief Helper function to reconstruct path from end to start
     */
    template<typename CoordType>
    std::vector<std::tuple<CoordType, CoordType>> reconstructPath(
        Pathfinding_Grid<CoordType>& grid,
        const std::tuple<CoordType, CoordType>& start,
        const std::tuple<CoordType, CoordType>& end)
    {
        std::vector<std::tuple<CoordType, CoordType>> path;
        auto pathNode = end;

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

    /**
     * @brief A* algorithm - Optimal pathfinding using heuristic
     */
    template<typename CoordType>
    std::vector<std::tuple<CoordType, CoordType>> findPathAStar(
        Pathfinding_Grid<CoordType>& grid,
        const std::tuple<CoordType, CoordType>& start,
        const std::tuple<CoordType, CoordType>& end,
        HeuristicType heuristic = HeuristicType::Manhattan)
    {
        std::vector<std::tuple<CoordType, CoordType>> path;
        path.reserve(32);

        // Check if start and end are valid and walkable
        if (!grid.isWithinBounds(start) || !grid.isWithinBounds(end))
        {
            return path;
        }

        if (!grid.getNode(start).getIsWalkable() || !grid.getNode(end).getIsWalkable())
        {
            return path;
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
        startNode.parent = std::make_tuple(-1, -1);

        // Priority queue for open set
        std::priority_queue<std::tuple<CoordType, CoordType>,
                           std::vector<std::tuple<CoordType, CoordType>>,
                           NodeComparator<CoordType>> openSet{NodeComparator<CoordType>(&grid)};

        std::unordered_set<std::tuple<CoordType, CoordType>, CoordHash<CoordType>> openSetTracker;
        std::unordered_set<std::tuple<CoordType, CoordType>, CoordHash<CoordType>> closedSet;

        openSet.push(start);
        openSetTracker.insert(start);

        while (!openSet.empty())
        {
            auto current = openSet.top();
            openSet.pop();

            openSetTracker.erase(current);
            closedSet.insert(current);

            if (current == end)
            {
                return reconstructPath(grid, start, end);
            }

            const auto neighbors = grid.getNeighbors(current);
            for (const auto& neighbor : neighbors)
            {
                if (!grid.getNode(neighbor).getIsWalkable() ||
                    closedSet.find(neighbor) != closedSet.end())
                {
                    continue;
                }

                CoordType tentativeGCost = grid.getNode(current).gCost + 1;
                auto& neighborNode = grid.getNode(neighbor);

                if (openSetTracker.find(neighbor) == openSetTracker.end() ||
                    tentativeGCost < neighborNode.gCost)
                {
                    neighborNode.parent = current;
                    neighborNode.gCost = tentativeGCost;
                    neighborNode.hCost = heuristicFunc(neighbor, end);
                    neighborNode.calculateFCost();

                    if (openSetTracker.find(neighbor) == openSetTracker.end())
                    {
                        openSet.push(neighbor);
                        openSetTracker.insert(neighbor);
                    }
                }
            }
        }

        return path; // Empty path - no path found
    }

    /**
     * @brief Dijkstra's algorithm - Optimal pathfinding without heuristic
     */
    template<typename CoordType>
    std::vector<std::tuple<CoordType, CoordType>> findPathDijkstra(
        Pathfinding_Grid<CoordType>& grid,
        const std::tuple<CoordType, CoordType>& start,
        const std::tuple<CoordType, CoordType>& end)
    {
        std::vector<std::tuple<CoordType, CoordType>> path;
        path.reserve(32);

        if (!grid.isWithinBounds(start) || !grid.isWithinBounds(end))
        {
            return path;
        }

        if (!grid.getNode(start).getIsWalkable() || !grid.getNode(end).getIsWalkable())
        {
            return path;
        }

        if (start == end)
        {
            path.push_back(start);
            return path;
        }

        // Reset grid to clear any previous pathfinding state
        grid.reset();

        // Initialize start node
        auto& startNode = grid.getNode(start);
        startNode.gCost = 0;
        startNode.parent = std::make_tuple(-1, -1);

        // Priority queue based on gCost only
        auto comparator = [&grid](const auto& a, const auto& b) {
            return grid.getNode(a).gCost > grid.getNode(b).gCost;
        };

        std::priority_queue<std::tuple<CoordType, CoordType>,
                           std::vector<std::tuple<CoordType, CoordType>>,
                           decltype(comparator)> openSet(comparator);

        std::unordered_set<std::tuple<CoordType, CoordType>, CoordHash<CoordType>> visited;

        openSet.push(start);

        while (!openSet.empty())
        {
            auto current = openSet.top();
            openSet.pop();

            if (visited.find(current) != visited.end())
            {
                continue;
            }

            visited.insert(current);

            if (current == end)
            {
                return reconstructPath(grid, start, end);
            }

            const auto neighbors = grid.getNeighbors(current);
            for (const auto& neighbor : neighbors)
            {
                if (!grid.getNode(neighbor).getIsWalkable() ||
                    visited.find(neighbor) != visited.end())
                {
                    continue;
                }

                CoordType tentativeGCost = grid.getNode(current).gCost + 1;
                auto& neighborNode = grid.getNode(neighbor);

                if (neighborNode.parent == std::make_tuple(-1, -1) ||
                    tentativeGCost < neighborNode.gCost)
                {
                    neighborNode.parent = current;
                    neighborNode.gCost = tentativeGCost;
                    openSet.push(neighbor);
                }
            }
        }

        return path;
    }

    /**
     * @brief Breadth-First Search - Optimal for unweighted graphs
     */
    template<typename CoordType>
    std::vector<std::tuple<CoordType, CoordType>> findPathBFS(
        Pathfinding_Grid<CoordType>& grid,
        const std::tuple<CoordType, CoordType>& start,
        const std::tuple<CoordType, CoordType>& end)
    {
        std::vector<std::tuple<CoordType, CoordType>> path;
        path.reserve(32);

        if (!grid.isWithinBounds(start) || !grid.isWithinBounds(end))
        {
            return path;
        }

        if (!grid.getNode(start).getIsWalkable() || !grid.getNode(end).getIsWalkable())
        {
            return path;
        }

        if (start == end)
        {
            path.push_back(start);
            return path;
        }

        std::queue<std::tuple<CoordType, CoordType>> queue;
        std::unordered_set<std::tuple<CoordType, CoordType>, CoordHash<CoordType>> visited;

        grid.getNode(start).parent = std::make_tuple(-1, -1);
        queue.push(start);
        visited.insert(start);

        while (!queue.empty())
        {
            auto current = queue.front();
            queue.pop();

            if (current == end)
            {
                return reconstructPath(grid, start, end);
            }

            const auto neighbors = grid.getNeighbors(current);
            for (const auto& neighbor : neighbors)
            {
                if (!grid.getNode(neighbor).getIsWalkable() ||
                    visited.find(neighbor) != visited.end())
                {
                    continue;
                }

                grid.getNode(neighbor).parent = current;
                queue.push(neighbor);
                visited.insert(neighbor);
            }
        }

        return path;
    }

    /**
     * @brief Depth-First Search - Fast but non-optimal
     */
    template<typename CoordType>
    std::vector<std::tuple<CoordType, CoordType>> findPathDFS(
        Pathfinding_Grid<CoordType>& grid,
        const std::tuple<CoordType, CoordType>& start,
        const std::tuple<CoordType, CoordType>& end)
    {
        std::vector<std::tuple<CoordType, CoordType>> path;
        path.reserve(32);

        if (!grid.isWithinBounds(start) || !grid.isWithinBounds(end))
        {
            return path;
        }

        if (!grid.getNode(start).getIsWalkable() || !grid.getNode(end).getIsWalkable())
        {
            return path;
        }

        if (start == end)
        {
            path.push_back(start);
            return path;
        }

        std::stack<std::tuple<CoordType, CoordType>> stack;
        std::unordered_set<std::tuple<CoordType, CoordType>, CoordHash<CoordType>> visited;

        grid.getNode(start).parent = std::make_tuple(-1, -1);
        stack.push(start);

        while (!stack.empty())
        {
            auto current = stack.top();
            stack.pop();

            if (visited.find(current) != visited.end())
            {
                continue;
            }

            visited.insert(current);

            if (current == end)
            {
                return reconstructPath(grid, start, end);
            }

            const auto neighbors = grid.getNeighbors(current);
            for (const auto& neighbor : neighbors)
            {
                if (!grid.getNode(neighbor).getIsWalkable() ||
                    visited.find(neighbor) != visited.end())
                {
                    continue;
                }

                auto& neighborNode = grid.getNode(neighbor);
                if (neighborNode.parent == std::make_tuple(-1, -1))
                {
                    neighborNode.parent = current;
                }
                stack.push(neighbor);
            }
        }

        return path;
    }

    /**
     * @brief Greedy Best-First Search - Fast but non-optimal, uses heuristic only
     */
    template<typename CoordType>
    std::vector<std::tuple<CoordType, CoordType>> findPathGreedyBestFirst(
        Pathfinding_Grid<CoordType>& grid,
        const std::tuple<CoordType, CoordType>& start,
        const std::tuple<CoordType, CoordType>& end,
        HeuristicType heuristic = HeuristicType::Manhattan)
    {
        std::vector<std::tuple<CoordType, CoordType>> path;
        path.reserve(32);

        if (!grid.isWithinBounds(start) || !grid.isWithinBounds(end))
        {
            return path;
        }

        if (!grid.getNode(start).getIsWalkable() || !grid.getNode(end).getIsWalkable())
        {
            return path;
        }

        if (start == end)
        {
            path.push_back(start);
            return path;
        }

        // Reset grid to clear any previous pathfinding state
        grid.reset();

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
        startNode.hCost = heuristicFunc(start, end);
        startNode.parent = std::make_tuple(-1, -1);

        std::priority_queue<std::tuple<CoordType, CoordType>,
                           std::vector<std::tuple<CoordType, CoordType>>,
                           GreedyComparator<CoordType>> openSet{GreedyComparator<CoordType>(&grid)};

        std::unordered_set<std::tuple<CoordType, CoordType>, CoordHash<CoordType>> visited;

        openSet.push(start);

        while (!openSet.empty())
        {
            auto current = openSet.top();
            openSet.pop();

            if (visited.find(current) != visited.end())
            {
                continue;
            }

            visited.insert(current);

            if (current == end)
            {
                return reconstructPath(grid, start, end);
            }

            const auto neighbors = grid.getNeighbors(current);
            for (const auto& neighbor : neighbors)
            {
                if (!grid.getNode(neighbor).getIsWalkable() ||
                    visited.find(neighbor) != visited.end())
                {
                    continue;
                }

                auto& neighborNode = grid.getNode(neighbor);
                if (neighborNode.parent == std::make_tuple(-1, -1))
                {
                    neighborNode.parent = current;
                    neighborNode.hCost = heuristicFunc(neighbor, end);
                    openSet.push(neighbor);
                }
            }
        }

        return path;
    }

    /**
     * @brief Bidirectional A* - Searches from both start and end simultaneously
     */
    template<typename CoordType>
    std::vector<std::tuple<CoordType, CoordType>> findPathBidirectionalAStar(
        Pathfinding_Grid<CoordType>& grid,
        const std::tuple<CoordType, CoordType>& start,
        const std::tuple<CoordType, CoordType>& end,
        HeuristicType heuristic = HeuristicType::Manhattan)
    {
        std::vector<std::tuple<CoordType, CoordType>> path;
        path.reserve(32);

        if (!grid.isWithinBounds(start) || !grid.isWithinBounds(end))
        {
            return path;
        }

        if (!grid.getNode(start).getIsWalkable() || !grid.getNode(end).getIsWalkable())
        {
            return path;
        }

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

        // Maps to store parent pointers for both directions
        std::unordered_map<std::tuple<CoordType, CoordType>, std::tuple<CoordType, CoordType>, CoordHash<CoordType>> parentFromStart;
        std::unordered_map<std::tuple<CoordType, CoordType>, std::tuple<CoordType, CoordType>, CoordHash<CoordType>> parentFromEnd;

        std::unordered_map<std::tuple<CoordType, CoordType>, CoordType, CoordHash<CoordType>> gCostFromStart;
        std::unordered_map<std::tuple<CoordType, CoordType>, CoordType, CoordHash<CoordType>> gCostFromEnd;

        std::unordered_set<std::tuple<CoordType, CoordType>, CoordHash<CoordType>> closedFromStart;
        std::unordered_set<std::tuple<CoordType, CoordType>, CoordHash<CoordType>> closedFromEnd;

        // Lambda for creating priority queue comparator
        auto makeComparator = [&](const auto& gCostMap, const auto& target) {
            return [&gCostMap, &target, &heuristicFunc](const auto& a, const auto& b) {
                auto it_a = gCostMap.find(a);
                auto it_b = gCostMap.find(b);
                CoordType g_a = (it_a != gCostMap.end()) ? it_a->second : 0;
                CoordType g_b = (it_b != gCostMap.end()) ? it_b->second : 0;
                CoordType f_a = g_a + heuristicFunc(a, target);
                CoordType f_b = g_b + heuristicFunc(b, target);
                return f_a > f_b; // Min-heap
            };
        };

        auto compFromStart = makeComparator(gCostFromStart, end);
        auto compFromEnd = makeComparator(gCostFromEnd, start);

        std::priority_queue<std::tuple<CoordType, CoordType>,
                           std::vector<std::tuple<CoordType, CoordType>>,
                           decltype(compFromStart)> openFromStart(compFromStart);

        std::priority_queue<std::tuple<CoordType, CoordType>,
                           std::vector<std::tuple<CoordType, CoordType>>,
                           decltype(compFromEnd)> openFromEnd(compFromEnd);

        parentFromStart[start] = std::make_tuple(-1, -1);
        parentFromEnd[end] = std::make_tuple(-1, -1);
        gCostFromStart[start] = 0;
        gCostFromEnd[end] = 0;

        openFromStart.push(start);
        openFromEnd.push(end);

        std::tuple<CoordType, CoordType> meetingPoint = std::make_tuple(-1, -1);
        bool pathFound = false;

        while (!openFromStart.empty() && !openFromEnd.empty())
        {
            // Expand from start
            if (!openFromStart.empty())
            {
                auto current = openFromStart.top();
                openFromStart.pop();

                if (closedFromStart.find(current) != closedFromStart.end())
                {
                    continue;
                }

                closedFromStart.insert(current);

                // Check if this node was visited from the other direction
                if (closedFromEnd.find(current) != closedFromEnd.end())
                {
                    meetingPoint = current;
                    pathFound = true;
                    break;
                }

                const auto neighbors = grid.getNeighbors(current);
                for (const auto& neighbor : neighbors)
                {
                    if (!grid.getNode(neighbor).getIsWalkable() ||
                        closedFromStart.find(neighbor) != closedFromStart.end())
                    {
                        continue;
                    }

                    CoordType tentativeGCost = gCostFromStart[current] + 1;

                    if (gCostFromStart.find(neighbor) == gCostFromStart.end() ||
                        tentativeGCost < gCostFromStart[neighbor])
                    {
                        parentFromStart[neighbor] = current;
                        gCostFromStart[neighbor] = tentativeGCost;
                        openFromStart.push(neighbor);
                    }
                }
            }

            // Expand from end
            if (!openFromEnd.empty())
            {
                auto current = openFromEnd.top();
                openFromEnd.pop();

                if (closedFromEnd.find(current) != closedFromEnd.end())
                {
                    continue;
                }

                closedFromEnd.insert(current);

                // Check if this node was visited from the other direction
                if (closedFromStart.find(current) != closedFromStart.end())
                {
                    meetingPoint = current;
                    pathFound = true;
                    break;
                }

                const auto neighbors = grid.getNeighbors(current);
                for (const auto& neighbor : neighbors)
                {
                    if (!grid.getNode(neighbor).getIsWalkable() ||
                        closedFromEnd.find(neighbor) != closedFromEnd.end())
                    {
                        continue;
                    }

                    CoordType tentativeGCost = gCostFromEnd[current] + 1;

                    if (gCostFromEnd.find(neighbor) == gCostFromEnd.end() ||
                        tentativeGCost < gCostFromEnd[neighbor])
                    {
                        parentFromEnd[neighbor] = current;
                        gCostFromEnd[neighbor] = tentativeGCost;
                        openFromEnd.push(neighbor);
                    }
                }
            }
        }

        if (!pathFound)
        {
            return path; // Empty path
        }

        // Reconstruct path from both directions
        std::vector<std::tuple<CoordType, CoordType>> pathFromStart;
        auto node = meetingPoint;
        while (!(std::get<0>(parentFromStart[node]) == -1 && std::get<1>(parentFromStart[node]) == -1))
        {
            pathFromStart.push_back(node);
            node = parentFromStart[node];
        }
        pathFromStart.push_back(start);
        std::reverse(pathFromStart.begin(), pathFromStart.end());

        std::vector<std::tuple<CoordType, CoordType>> pathFromEnd;
        node = parentFromEnd[meetingPoint];
        while (node != end && !(std::get<0>(parentFromEnd[node]) == -1 && std::get<1>(parentFromEnd[node]) == -1))
        {
            pathFromEnd.push_back(node);
            node = parentFromEnd[node];
        }
        if (node == end)
        {
            pathFromEnd.push_back(end);
        }

        // Combine paths
        path = pathFromStart;
        path.insert(path.end(), pathFromEnd.begin(), pathFromEnd.end());

        return path;
    }

    /**
     * @brief Unified findPath function that supports all algorithms
     *
     * @param grid The grid to search on
     * @param start Starting position
     * @param end Target position
     * @param algorithm Algorithm to use
     * @param heuristic Heuristic function (for algorithms that use it)
     * @return Vector of coordinates representing the path (empty if no path found)
     */
    template<typename CoordType>
    std::vector<std::tuple<CoordType, CoordType>> findPath(
        Pathfinding_Grid<CoordType>& grid,
        const std::tuple<CoordType, CoordType>& start,
        const std::tuple<CoordType, CoordType>& end,
        Algorithm algorithm = Algorithm::AStar,
        HeuristicType heuristic = HeuristicType::Manhattan)
    {
        switch (algorithm)
        {
            case Algorithm::AStar:
                return findPathAStar(grid, start, end, heuristic);
            case Algorithm::Dijkstra:
                return findPathDijkstra(grid, start, end);
            case Algorithm::BFS:
                return findPathBFS(grid, start, end);
            case Algorithm::DFS:
                return findPathDFS(grid, start, end);
            case Algorithm::GreedyBestFirst:
                return findPathGreedyBestFirst(grid, start, end, heuristic);
            case Algorithm::BidirectionalAStar:
                return findPathBidirectionalAStar(grid, start, end, heuristic);
            default:
                return findPathAStar(grid, start, end, heuristic);
        }
    }

    /**
     * @brief Convenience function to find path on a simple grid
     *
     * @param width Grid width
     * @param height Grid height
     * @param obstacles Set of obstacle coordinates
     * @param start Starting position
     * @param end Target position
     * @param algorithm Algorithm to use
     * @param heuristic Heuristic function (for algorithms that use it)
     * @return Vector of coordinates representing the path
     */
    template<typename CoordType>
    std::vector<std::tuple<CoordType, CoordType>> findPathSimple(
        CoordType width, CoordType height,
        const std::vector<std::tuple<CoordType, CoordType>>& obstacles,
        const std::tuple<CoordType, CoordType>& start,
        const std::tuple<CoordType, CoordType>& end,
        Algorithm algorithm = Algorithm::AStar,
        HeuristicType heuristic = HeuristicType::Manhattan)
    {
        Pathfinding_Grid<CoordType> grid(width, height);

        for (const auto& obstacle : obstacles)
        {
            grid.setObstacle(std::get<0>(obstacle), std::get<1>(obstacle));
        }

        return findPath(grid, start, end, algorithm, heuristic);
    }
}
