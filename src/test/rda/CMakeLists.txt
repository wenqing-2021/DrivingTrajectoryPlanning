cmake_minimum_required(VERSION 3.10)

# RDA Planner Tests

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
