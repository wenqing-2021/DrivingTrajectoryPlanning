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
