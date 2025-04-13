project(QPSolver)

# Add source files and header files
file(GLOB QPSOLVERS_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(QPSolver
    ${QPSOLVERS_HEADERS}
)
set_property(TARGET QPSolver PROPERTY POSITION_INDEPENDENT_CODE ON)

# Include directories for headers
target_include_directories(QPSolver PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(QPSolver PUBLIC ${UTILS_LIBRARIES})