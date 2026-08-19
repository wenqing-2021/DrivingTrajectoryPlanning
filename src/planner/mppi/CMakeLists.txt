add_library(MppiPlanner mppi_planner.cpp mppi_planner.h)
set_property(TARGET MppiPlanner PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(MppiPlanner PUBLIC ${CMAKE_SOURCE_DIR}/src)

pybind11_add_module(mppi_pybind mppi_pybind.cpp)
target_link_libraries(mppi_pybind PRIVATE MppiPlanner)

set(
    PYBIND_LIBRARIES
    ${PYBIND_LIBRARIES}
    mppi_pybind
    CACHE INTERNAL "LIST FOR PYBIND11 LIBRARIES" FORCE
)
