get_filename_component(x3_proto "./protos/x3.proto" ABSOLUTE)
get_filename_component(x3_proto_path "${sample_proto}" PATH)
get_filename_component(sample_proto "./protos/sample.proto" ABSOLUTE)
get_filename_component(sample_proto_path "${sample_proto}" PATH)

set(sample_proto_sources "${CMAKE_CURRENT_BINARY_DIR}/sample.pb.cc")
set(sample_proto_headers "${CMAKE_CURRENT_BINARY_DIR}/sample.pb.h")
set(sample_grpc_sources "${CMAKE_CURRENT_BINARY_DIR}/sample.grpc.pb.cc")
set(sample_grpc_headers "${CMAKE_CURRENT_BINARY_DIR}/sample.grpc.pb.h")
set(x3_proto_sources "${CMAKE_CURRENT_BINARY_DIR}/x3.pb.cc")
set(x3_proto_headers "${CMAKE_CURRENT_BINARY_DIR}/x3.pb.h")
set(x3_grpc_sources "${CMAKE_CURRENT_BINARY_DIR}/x3.grpc.pb.cc")
set(x3_grpc_headers "${CMAKE_CURRENT_BINARY_DIR}/x3.grpc.pb.h")
add_custom_command(
        OUTPUT "${sample_proto_sources}" "${sample_proto_headers}" "${sample_grpc_sources}" "${sample_grpc_headers}"
        "${x3_proto_sources}" "${x3_proto_headers}" "${x3_grpc_sources}" "${x3_grpc_headers}"
        COMMAND ${_PROTOBUF_PROTOC}
        ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}" --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${sample_proto_path}" "${x3_proto_path}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${sample_proto}" "${x3_proto}"
        DEPENDS "${sample_proto}" "${x3_proto}")

include_directories("${CMAKE_CURRENT_BINARY_DIR}")

add_library(sample_grpc_proto
        ${sample_proto_sources}
        ${sample_proto_headers}
        ${sample_grpc_sources}
        ${sample_grpc_headers}
        ${x3_proto_sources}
        ${x3_proto_headers}
        ${x3_grpc_sources}
        ${x3_grpc_headers})
target_link_libraries(sample_grpc_proto
        ${_PROTOBUF_LIBPROTOBUF}
        ${_REFLECTION}
        ${_GRPC_GRPCPP})

install(TARGETS
  sample_grpc_proto
  DESTINATION lib/${PROJECT_NAME})
  
message("CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR}")
# install(
#   FILES 
#   ${CMAKE_CURRENT_BINARY_DIR}/sample.grpc.pb.cc  
#   ${CMAKE_CURRENT_BINARY_DIR}/sample.pb.cc 
#   ${CMAKE_CURRENT_BINARY_DIR}/sample.grpc.pb.h   
#   ${CMAKE_CURRENT_BINARY_DIR}/sample.pb.h
#   DESTINATION include/${PROJECT_NAME}/
# )
