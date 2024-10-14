include_directories("${CMAKE_CURRENT_BINARY_DIR}/../proto_compiling")
link_directories("${CMAKE_CURRENT_BINARY_DIR}/../proto_compiling")

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
  ${CMAKE_SYSROOT}/usr/include/opencv4
)

# Add sources
set(SOURCES
  src/main.cpp
)

# Create executable
add_executable(grpc_server ${SOURCES})

target_link_libraries(grpc_server
        sample_grpc_proto
        ${_PROTOBUF_LIBPROTOBUF}
        ${_REFLECTION}
        ${_GRPC_GRPCPP}
        ${OPENCV_LIB}
)

# Add dependencies
ament_target_dependencies(grpc_server 
  rclcpp 
  sensor_msgs 
  ai_msgs 
)

# Install target
install(TARGETS
  grpc_server
  DESTINATION lib/${PROJECT_NAME})
