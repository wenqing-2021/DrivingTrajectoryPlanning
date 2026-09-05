
add_executable(TestCppADIpopt cppad_ipopt_demo.cpp)
target_link_libraries(TestCppADIpopt PRIVATE CppAD::CppAD PkgConfig::IPOPT)
