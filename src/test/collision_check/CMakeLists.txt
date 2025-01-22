# Source files
file(GLOB COLLISION_CHECK_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

# Add test executable
add_executable(TestCollisionCheck ${COLLISION_CHECK_SRC})

target_link_libraries(TestCollisionCheck PUBLIC ${UTILS_LIBRARIES} ${GTEST_LIBRARIES})