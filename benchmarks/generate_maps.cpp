#include "../AStarLib.hpp"
#include <iostream>
#include <fstream>
#include <random>
#include <set>

using namespace AStarLib;

// Helper to generate verified obstacle map
std::vector<std::tuple<int, int>> generateVerifiedMap(
    int width, int height,
    std::tuple<int, int> start,
    std::tuple<int, int> end,
    int targetObstacles,
    int seed) {

    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> distX(0, width - 1);
    std::uniform_int_distribution<int> distY(0, height - 1);

    AStar_Grid<int> grid(width, height);
    std::vector<std::tuple<int, int>> obstacles;
    std::set<std::tuple<int, int>> tried;

    std::cout << "Generating " << targetObstacles << " obstacles with seed " << seed << "...\n";

    int attempts = 0;
    int maxAttempts = targetObstacles * 10;

    while (obstacles.size() < static_cast<size_t>(targetObstacles) && attempts < maxAttempts) {
        int x = distX(rng);
        int y = distY(rng);
        auto pos = std::make_tuple(x, y);

        // Skip if already tried or is start/end
        if (tried.count(pos) || pos == start || pos == end) {
            attempts++;
            continue;
        }

        tried.insert(pos);

        // Try adding obstacle
        grid.setObstacle(x, y);

        // Verify path still exists
        auto path = findPath(grid, start, end);
        if (!path.empty()) {
            obstacles.push_back(pos);
            if (obstacles.size() % 100 == 0) {
                std::cout << "  Added " << obstacles.size() << " obstacles...\n";
            }
        } else {
            // Revert if path blocked
            grid.setWalkable(x, y);
        }

        attempts++;
    }

    // Verify final path
    auto finalPath = findPath(grid, start, end);
    std::cout << "Final: " << obstacles.size() << " obstacles, path length: " << finalPath.size() << "\n\n";

    return obstacles;
}

void writeMapToFile(std::ofstream& out, const std::string& varName,
                    const std::vector<std::tuple<int, int>>& obstacles,
                    const std::string& comment) {
    out << "// " << comment << "\n";
    out << "const std::vector<std::tuple<int, int>> " << varName << " = {\n";

    for (size_t i = 0; i < obstacles.size(); ++i) {
        if (i % 8 == 0) out << "    ";
        out << "{" << std::get<0>(obstacles[i]) << ", " << std::get<1>(obstacles[i]) << "}";
        if (i < obstacles.size() - 1) out << ", ";
        if (i % 8 == 7 || i == obstacles.size() - 1) out << "\n";
    }

    out << "};\n\n";
}

int main() {
    std::ofstream out("obstacle_maps.hpp");

    out << "#pragma once\n";
    out << "#include <vector>\n";
    out << "#include <tuple>\n\n";
    out << "/**\n";
    out << " * @brief Pre-generated obstacle maps with verified paths for benchmarking\n";
    out << " *\n";
    out << " * These maps have been generated and verified to have valid paths from (0,0) to (99,99).\n";
    out << " * This ensures consistent and meaningful benchmark results.\n";
    out << " */\n";
    out << "namespace ObstacleMaps {\n\n";

    auto start = std::make_tuple(0, 0);
    auto end = std::make_tuple(99, 99);

    // Generate 10% obstacles
    auto map10 = generateVerifiedMap(100, 100, start, end, 1000, 12345);
    writeMapToFile(out, "obstacles_100x100_10percent", map10,
                   "100x100 grid with ~10% obstacles (" + std::to_string(map10.size()) + " obstacles, path verified)");

    // Generate 20% obstacles
    auto map20 = generateVerifiedMap(100, 100, start, end, 2000, 54321);
    writeMapToFile(out, "obstacles_100x100_20percent", map20,
                   "100x100 grid with ~20% obstacles (" + std::to_string(map20.size()) + " obstacles, path verified)");

    // Generate 30% obstacles
    auto map30 = generateVerifiedMap(100, 100, start, end, 3000, 98765);
    writeMapToFile(out, "obstacles_100x100_30percent", map30,
                   "100x100 grid with ~30% obstacles (" + std::to_string(map30.size()) + " obstacles, path verified)");

    out << "} // namespace ObstacleMaps\n";
    out.close();

    std::cout << "Generated obstacle_maps.hpp successfully!\n";

    return 0;
}
