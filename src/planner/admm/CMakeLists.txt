project(ADMMSolver)

# Add source files and header files
file(GLOB ADMMSOLVERS_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
file(GLOB ADMMSOLVERS_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_library(ADMMSolver
    ${ADMMSOLVERS_SRC}
    ${ADMMSOLVERS_HEADERS}
)
set_property(TARGET ADMMSolver PROPERTY POSITION_INDEPENDENT_CODE ON)

# Include directories for headers
target_include_directories(ADMMSolver PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(ADMMSolver PUBLIC ${UTILS_LIBRARIES})