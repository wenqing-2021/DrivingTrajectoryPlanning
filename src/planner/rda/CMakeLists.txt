project(RDAPlanner)

# Add source files and header files
file(GLOB RDAPlanner_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
file(GLOB RDAPlanner_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_library(RDAPlanner
    ${RDAPlanner_SRC}
    ${RDAPlanner_HEADERS}
)
set_property(TARGET RDAPlanner PROPERTY POSITION_INDEPENDENT_CODE ON)

# Include directories for headers
target_include_directories(RDAPlanner PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(RDAPlanner PUBLIC ${UTILS_LIBRARIES} Map)