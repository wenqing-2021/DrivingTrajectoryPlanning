project(PiecewiseJerkSpeedOptimizer)

# Add source files and header files
file(GLOB PWJ_SPEED_OPTIMIZER_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB PWJ_SPEED_OPTIMIZER_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(PiecewiseJerkSpeedOptimizer
    ${PWJ_SPEED_OPTIMIZER_SRC}
    ${PWJ_SPEED_OPTIMIZER_HEADERS}
)
set_property(TARGET PiecewiseJerkSpeedOptimizer PROPERTY POSITION_INDEPENDENT_CODE ON)

# Include directories for headers
target_include_directories(PiecewiseJerkSpeedOptimizer PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(PiecewiseJerkSpeedOptimizer PUBLIC ${UTILS_LIBRARIES})