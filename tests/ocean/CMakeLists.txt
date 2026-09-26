add_executable(test_ocean_math test_ocean_math.cpp)
target_link_libraries(test_ocean_math PRIVATE OCEANPlanner)
add_test(NAME test_ocean_math COMMAND test_ocean_math)

# End-to-end regression: serial and parallel rounds must reproduce the same
# quality-validated result across repeated requests on one persistent Solver.
# Each run also dumps per-case protobuf payloads next to the JSON, so keep the
# whole result set inside one directory instead of the build root.
set(OCEAN_BENCHMARK_RESULT_DIRECTORY "${CMAKE_BINARY_DIR}/ocean/solve_res_test"
    CACHE PATH "Directory receiving the OCEAN benchmark JSON, Markdown and per-case payloads")
foreach(ocean_workers 1 4)
    add_test(NAME test_ocean_cases_${ocean_workers}
        COMMAND "${Python_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tests/benchmark/benchmark_planners.py"
            --backend ocean --cases 1 13 --repeats 2 --workers ${ocean_workers}
            --output "${OCEAN_BENCHMARK_RESULT_DIRECTORY}/ocean_cases_${ocean_workers}.json")
    set_tests_properties(test_ocean_cases_${ocean_workers} PROPERTIES TIMEOUT 180)
endforeach()

add_executable(test_ocean_parameters test_ocean_parameters.cpp)
target_link_libraries(test_ocean_parameters PRIVATE OCEANPlanner Map ${UTILS_LIBRARIES})
add_test(NAME test_ocean_parameters COMMAND test_ocean_parameters)
