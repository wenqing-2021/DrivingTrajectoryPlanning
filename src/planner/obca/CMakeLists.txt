project(OBCASolver)

# Add source files and header files
file(GLOB OBCASOLVERS_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
file(GLOB OBCASOLVERS_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_library(OBCASolver
    ${OBCASOLVERS_SRC}
    ${OBCASOLVERS_HEADERS}
)
set_property(TARGET OBCASolver PROPERTY POSITION_INDEPENDENT_CODE ON)

# Include directories for headers
target_include_directories(OBCASolver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(OBCASolver PUBLIC ${UTILS_LIBRARIES} Map)