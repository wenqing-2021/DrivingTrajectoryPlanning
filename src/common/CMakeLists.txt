project(COMMON)

# Add source files and header files
file(GLOB MATH_SRC "${CMAKE_CURRENT_SOURCE_DIR}/math/*.cc")
file(GLOB MATH_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/math/*.h")

# Create interface library for common include directories
# add_library(CommonHeaders INTERFACE)
# target_include_directories(CommonHeaders INTERFACE
#     ${CMAKE_SOURCE_DIR}/src
# )

# Create actual libraries
add_library(Common STATIC ${MATH_SRC})
target_include_directories(Common PUBLIC
    ${CMAKE_SOURCE_DIR}/src/common
    # ${CMAKE_CURRENT_SOURCE_DIR}/math
    # ${CMAKE_CURRENT_SOURCE_DIR}/qp_solver
)
# Link interface library to actual libraries
target_link_libraries(Common PUBLIC
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
    ProtoLib
)

# Uncomment if position-independent code is needed
# set_property(TARGET Math PROPERTY POSITION_INDEPENDENT_CODE ON)

# set(UTILS_LIBRARIES ${UTILS_LIBRARIES} Math PARENT_SCOPE)