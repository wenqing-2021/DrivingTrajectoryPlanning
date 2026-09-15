# RDA Planner Tests

add_executable(test_trajectory_controls test_trajectory_controls.cpp)
target_link_libraries(test_trajectory_controls PRIVATE Planner)
add_test(NAME test_trajectory_controls COMMAND test_trajectory_controls)

add_executable(test_rda_main
    test_rda_main.cpp
)

target_link_libraries(test_rda_main
    PUBLIC
    Map ${UTILS_LIBRARIES} Planner
)

target_include_directories(test_rda_main
    PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/src/common
    ${PROJECT_SOURCE_DIR}/src/planner
    ${PROJECT_SOURCE_DIR}/src/map
    ${PROJECT_BINARY_DIR}
)

add_test(
    NAME test_rda_main
    COMMAND test_rda_main
    WORKING_DIRECTORY /tmp
)

# Case2 reproduction test (diagonal parking)
add_executable(test_rda_case2
    test_rda_case2.cpp
)

target_link_libraries(test_rda_case2
    PUBLIC
    Map ${UTILS_LIBRARIES} Planner
)

target_include_directories(test_rda_case2
    PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/src/common
    ${PROJECT_SOURCE_DIR}/src/planner
    ${PROJECT_SOURCE_DIR}/src/map
    ${PROJECT_BINARY_DIR}
)

add_test(
    NAME test_rda_case2
    COMMAND test_rda_case2
    WORKING_DIRECTORY /tmp
)

add_executable(test_qp_status test_qp_status.cpp)
target_link_libraries(test_qp_status PRIVATE Planner)
add_test(NAME test_qp_status COMMAND test_qp_status)

add_executable(test_rda_coordinates test_rda_coordinates.cpp)
target_link_libraries(test_rda_coordinates PRIVATE Planner)
add_test(NAME test_rda_coordinates COMMAND test_rda_coordinates)

# Use this checkout's configuration even when CTest runs from /tmp.
foreach(rda_test test_rda_main test_rda_case2)
    target_compile_definitions(${rda_test} PRIVATE
        RDA_TEST_CONFIG_PATH="${PROJECT_SOURCE_DIR}/src/config/solver_params.yaml")
endforeach()

add_executable(test_rda_parameters test_rda_parameters.cpp)
target_link_libraries(test_rda_parameters PRIVATE Planner)
add_test(NAME test_rda_parameters COMMAND test_rda_parameters)

add_executable(test_rda_workspace test_rda_workspace.cpp)
target_link_libraries(test_rda_workspace PRIVATE RDAPlanner)
add_test(NAME test_rda_workspace COMMAND test_rda_workspace)

# Regression for adaptive OSQP state leaking into the next planning request.
add_test(NAME test_rda_repeated_request
    COMMAND "${Python_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tests/rda/benchmark_runtime.py"
        --cases 8 --repeats 2 --workers 4 --output "${CMAKE_BINARY_DIR}/runtime_case8.json")
set_tests_properties(test_rda_repeated_request PROPERTIES TIMEOUT 30)

add_executable(test_cone_thread_pool test_cone_thread_pool.cpp)
target_link_libraries(test_cone_thread_pool PRIVATE RDAPlanner)
add_test(NAME test_cone_thread_pool COMMAND test_cone_thread_pool)
set_tests_properties(test_cone_thread_pool PROPERTIES TIMEOUT 30)
