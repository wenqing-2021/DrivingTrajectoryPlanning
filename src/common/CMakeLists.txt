project(COMMON)

# Add source files and header files
file(GLOB MATH_SRC "${CMAKE_CURRENT_SOURCE_DIR}/math/*.cc")
file(GLOB MATH_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/math/*.h")
file(GLOB UTILS_SRC "${CMAKE_CURRENT_SOURCE_DIR}/util/*.cc")
file(GLOB UTILS_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/util/*.h")

# Create interface library for common include directories
add_library(CommonHeaders INTERFACE)
target_include_directories(CommonHeaders INTERFACE
    ${CMAKE_SOURCE_DIR}/src
)

# Create actual libraries
add_library(Math STATIC ${MATH_SRC})
add_library(Utils STATIC ${UTILS_SRC})

# Link interface library to actual libraries
target_link_libraries(Math PUBLIC CommonHeaders)
target_link_libraries(Utils PUBLIC CommonHeaders)

# Uncomment if position-independent code is needed
# set_property(TARGET Math PROPERTY POSITION_INDEPENDENT_CODE ON)
# set_property(TARGET Utils PROPERTY POSITION_INDEPENDENT_CODE ON)

set(UTILS_LIBRARIES ${UTILS_LIBRARIES} Math Utils PARENT_SCOPE)