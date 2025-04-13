# set the required version
cmake_minimum_required(VERSION 3.23.0)

# set the C++ version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set project name
project(AutomatedPark)

# find necessary packages
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Protobuf REQUIRED)
find_package(yaml-cpp REQUIRED)
# Find GoogleTest package
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
add_subdirectory(third-party/pybind11)
add_subdirectory(third-party/glog)
add_subdirectory(protobuf)
add_subdirectory(src/logger)
set(PYBIND_LIBRARIES "" CACHE INTERNAL "LIST FOR PYBIND11 LIBRARIES")
set(UTILS_LIBRARIES "" CACHE INTERNAL "LIST FOR UTILS LIBRARIES")
set(UTILS_LIBRARIES ${UTILS_LIBRARIES}
    Logger
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
    ProtoLib
    yaml-cpp
)

# add include directories
# add_subdirectory(src/forward_simulator)
# add_subdirectory(src/risk_assessment)
# add_subdirectory(src/ilqr_planner)
add_subdirectory(src/common)
add_subdirectory(src/collision_check)
add_subdirectory(src/map)
add_subdirectory(src/planner)
add_subdirectory(src/solver)

# BUILD PYBIND11 MODULE
# set the output directory
set(PYBIND_OUT_DIR ${CMAKE_BINARY_DIR}/pybind_modules)
message(STATUS "pybind11 output directory: ${PYBIND_OUT_DIR}")
message(STATUS "pybind11 libraries: ${PYBIND_LIBRARIES}")
FOREACH(pybind_lib ${PYBIND_LIBRARIES})
    message(STATUS "find pybind11 library: ${pybind_lib}")
    set_target_properties(${pybind_lib} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${PYBIND_OUT_DIR}
    )
ENDFOREACH()

install(
    DIRECTORY ./src/
    DESTINATION ${CMAKE_BINARY_DIR}/install/
    FILES_MATCHING PATTERN "*.py" PATTERN "*.yaml"
)

# add excutable
add_executable(DEBUG_SOLVER src/solver/debug_solver.cpp)
target_link_libraries(DEBUG_SOLVER PUBLIC Solver)

add_subdirectory(src/test/collision_check)