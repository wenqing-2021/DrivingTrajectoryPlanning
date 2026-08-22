project(TrajectoryPlanningSolver)

# Add source files and header files
file(GLOB SOLVER_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB SOLVER_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(Solver
    ${SOLVER_SOURCES}
    ${SOLVER_HEADERS}
)
set_property(TARGET Solver PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(Solver PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(Solver PUBLIC Map ${UTILS_LIBRARIES} Planner)

# add pybind11 module
file(GLOB SOLVER_PYBIND_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/pybind/*.cpp"
)
pybind11_add_module(solver_pybind ${SOLVER_PYBIND_SRC})
target_link_libraries(solver_pybind PUBLIC Solver)

set(PYBIND_LIBRARIES ${PYBIND_LIBRARIES} solver_pybind PARENT_SCOPE)
