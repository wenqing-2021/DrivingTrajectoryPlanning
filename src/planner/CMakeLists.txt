project(Planner)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/hybrid_a_star)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/speed_planner)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/rda)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/obca)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/mppi)

# Add source files and header files
file(GLOB PLANNER_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB PLANNER_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(Planner
    ${PLANNER_SRC}
    ${PLANNER_HEADERS}
)
set_property(TARGET Planner PROPERTY POSITION_INDEPENDENT_CODE ON)

# Include directories for headers
target_include_directories(Planner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Planner PUBLIC ${UTILS_LIBRARIES} HybridAStar PiecewiseJerkSpeedOptimizer RDAPlanner OBCASolver)
