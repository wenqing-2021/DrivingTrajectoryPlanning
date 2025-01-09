project(Map)

# Add source files and header files
file(GLOB COST_MAP_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB COST_MAP_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(Map
    ${COST_MAP_SRC}
    ${COST_MAP_HEADERS}
)
set_property(TARGET Map PROPERTY POSITION_INDEPENDENT_CODE ON)

# Include directories for headers
target_include_directories(Map PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(Map PUBLIC ${UTILS_LIBRARIES})