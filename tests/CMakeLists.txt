add_executable(tests
    test_node.cpp
    test_grid.cpp
    test_pathfinding.cpp
)

target_link_libraries(tests
    PRIVATE
        AStarLib
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(tests)
