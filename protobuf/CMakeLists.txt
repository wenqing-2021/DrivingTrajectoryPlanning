
# generate protobuf
file(GLOB PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
protobuf_generate_python(PROTO_PY ${PROTO_FILES})
add_library(ProtoLib ${PROTO_SRCS} ${PROTO_HDRS} ${PROTO_PY})
set_property(TARGET ProtoLib PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(ProtoLib PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(ProtoLib PUBLIC protobuf::protobuf)
install(
    FILES ${PROTO_PY}
    DESTINATION src/protobuf
)
