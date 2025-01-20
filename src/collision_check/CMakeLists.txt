project(CollisionCheck)

file(GLOB COLLISION_CHECK_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB COLLISION_CHECK_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(CollisionCheck ${COLLISION_CHECK_SRC})

target_include_directories(CollisionCheck PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(CollisionCheck PUBLIC ${UTILS_LIBRARIES})
set(UTILS_LIBRARIES ${UTILS_LIBRARIES} CollisionCheck PARENT_SCOPE)