project(HybridAStar)

# Add source files and header files
file(GLOB HYBRID_ASTAR_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB HYBRID_ASTAR_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(HybridAStar
    ${HYBRID_ASTAR_SRC}
    ${HYBRID_ASTAR_HEADERS}
)
set_property(TARGET HybridAStar PROPERTY POSITION_INDEPENDENT_CODE ON)

# Include directories for headers
target_include_directories(HybridAStar PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(HybridAStar PUBLIC ${UTILS_LIBRARIES})