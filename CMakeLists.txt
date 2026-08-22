# set the required version
cmake_minimum_required(VERSION 3.23.0)

# set the C++ version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

# set project name
project(TrajectoryPlanning)
enable_testing()

# find necessary packages
find_package(Eigen3 CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(glog CONFIG REQUIRED)
find_package(GTest CONFIG REQUIRED)
find_package(osqp CONFIG REQUIRED)
find_package(OsqpEigen 0.11.2 EXACT CONFIG REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Python COMPONENTS Interpreter Development REQUIRED)
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED IMPORTED_TARGET ipopt)
pkg_get_variable(IPOPT_PREFIX ipopt prefix)

find_path(CPPAD_INCLUDE_DIR cppad/cppad.hpp REQUIRED)
add_library(CppAD::CppAD INTERFACE IMPORTED)
set_target_properties(CppAD::CppAD PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CPPAD_INCLUDE_DIR};${IPOPT_PREFIX}/include"
)

add_subdirectory(third-party/EiCOS)
add_subdirectory(protobuf)
add_subdirectory(src/logger)
target_link_libraries(Logger PUBLIC yaml-cpp::yaml-cpp)
add_subdirectory(src/common)
set(PYBIND_LIBRARIES "" CACHE INTERNAL "LIST FOR PYBIND11 LIBRARIES")
set(UTILS_LIBRARIES "" CACHE INTERNAL "LIST FOR UTILS LIBRARIES")
set(UTILS_LIBRARIES ${UTILS_LIBRARIES}
    Logger
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
    ProtoLib
    yaml-cpp
    Common
    eicos
)

add_subdirectory(src/collision_check)
add_subdirectory(src/map)
add_subdirectory(src/planner)
add_subdirectory(src/solver)

# BUILD PYBIND11 MODULE
# set the output directory
set(PYBIND_OUT_DIR "${CMAKE_SOURCE_DIR}/build/pybind_modules")
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
    DESTINATION src/
    FILES_MATCHING
        PATTERN "*.py"
        PATTERN "*.yaml"
        PATTERN "__pycache__" EXCLUDE
)

# add excutable
add_executable(DEBUG_SOLVER src/solver/debug_solver.cpp)
target_link_libraries(DEBUG_SOLVER PUBLIC Solver)

add_subdirectory(src/test/collision_check)
add_subdirectory(src/test/cpp_ad)
add_subdirectory(src/test/obca)
add_subdirectory(src/test/rda)
add_subdirectory(src/test/mppi)
