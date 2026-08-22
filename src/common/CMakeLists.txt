project(COMMON)

# Add source files and header files
file(GLOB MATH_SRC "${CMAKE_CURRENT_SOURCE_DIR}/math/*.cc")

# Create actual libraries
add_library(Common STATIC ${MATH_SRC})
target_include_directories(Common PUBLIC
    ${CMAKE_SOURCE_DIR}/src/common
)
# Link interface library to actual libraries
target_link_libraries(Common PUBLIC
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
    CppAD::CppAD
    PkgConfig::IPOPT
    ProtoLib
    Logger
)
