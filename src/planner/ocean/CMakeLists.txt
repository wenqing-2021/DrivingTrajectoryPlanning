add_library(OCEANPlanner ocean_planner.cpp ocean_thread_pool.cpp)
set_property(TARGET OCEANPlanner PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(OCEANPlanner PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(OCEANPlanner PUBLIC ${UTILS_LIBRARIES} Map eicos OsqpEigen::OsqpEigen)
