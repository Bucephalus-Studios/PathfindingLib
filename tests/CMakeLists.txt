add_executable(astar_tests
    test_node.cpp
    test_grid.cpp
    test_pathfinding.cpp
)

target_link_libraries(astar_tests
    PRIVATE
        AStarLib
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(astar_tests)
