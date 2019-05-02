project(backcamera)

add_subdirectory(constraint_set)


file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  ArmorDetectionProtoSrc
  ArmorDetectionProtoHds
  ${ProtoFiles})


#backcamera node
add_executable(backcamera_node
  ${ArmorDetectionProtoSrc}
  ${ArmorDetectionProtoHds}
  backcamera_node.cpp
  gimbal_control.cpp
)

target_link_libraries(backcamera_node
  PRIVATE
  detection::tool_backcamera
  detection::constraint_set_backcamera
  ${PROTOBUF_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_include_directories(backcamera_node
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
)
add_dependencies(backcamera_node
  roborts_msgs_generate_messages)

#backcamera_client
add_executable(backcamera_client
  backcamera_client.cpp
)

target_include_directories(backcamera_client
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
  )

target_link_libraries(backcamera_client
  PRIVATE
  detection::tool_backcamera
  ${PROTOBUF_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(backcamera_client
  roborts_msgs_generate_messages)
