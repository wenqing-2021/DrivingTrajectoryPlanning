project(Logger)

# Add source files and header files
file(GLOB LOGGER_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB LOGGER_HEAD "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(Logger ${LOGGER_SRC} ${LOGGER_HEAD})

set_property(TARGET Logger PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(Logger PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(Logger PUBLIC glog::glog)
