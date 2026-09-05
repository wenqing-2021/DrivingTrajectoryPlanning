# OBCA Planner Tests

# C++ executable test for OBCA planner
add_executable(test_obca_main
    test_obca_main.cpp
)

# Link required libraries
target_link_libraries(test_obca_main
    PUBLIC
    Map ${UTILS_LIBRARIES} Planner
)

# Include directories
target_include_directories(test_obca_main
    PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/src/common
    ${PROJECT_SOURCE_DIR}/src/planner
    ${PROJECT_SOURCE_DIR}/src/map
    ${PROJECT_BINARY_DIR}
)

# Add test
add_test(
    NAME test_obca_main
    COMMAND test_obca_main
    WORKING_DIRECTORY /tmp
)
