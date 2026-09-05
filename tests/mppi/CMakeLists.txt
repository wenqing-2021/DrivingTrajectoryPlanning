add_executable(TestMppiPlanner test_mppi_planner.cpp)
target_link_libraries(TestMppiPlanner PRIVATE MppiPlanner GTest::gtest_main)
add_test(NAME TestMppiPlanner COMMAND TestMppiPlanner)
