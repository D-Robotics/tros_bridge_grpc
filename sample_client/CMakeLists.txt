include_directories("${CMAKE_CURRENT_BINARY_DIR}/../proto_compiling")
link_directories("${CMAKE_CURRENT_BINARY_DIR}/../proto_compiling")

include_directories(
  ${CMAKE_SYSROOT}/usr/include/opencv4
)

# foreach(_target sample_client sample_server sample_async_client sample_async_server)
foreach(_target sample_client)
    add_executable(${_target} "${_target}.cc")
    target_link_libraries(${_target}
            sample_grpc_proto
            ${_PROTOBUF_LIBPROTOBUF}
            ${_REFLECTION}
            ${_GRPC_GRPCPP}
            ${OpenCV_LIBS}
      )
    # Install target
    install(TARGETS
      ${_target}
      DESTINATION lib/${PROJECT_NAME})
endforeach()
