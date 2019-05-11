project(backcamera)

add_subdirectory(constraint_set)


file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  ArmorDetectionProtoSrc
  ArmorDetectionProtoHds
  ${ProtoFiles})

#apriltags
include_directories(include include/AprilTags)

add_library(apriltags
  src/Edge.cc src/FloatImage.cc src/GLine2D.cc src/GLineSegment2D.cc src/Gaussian.cc src/GrayModel.cc src/Homography33.cc src/MathUtil.cc src/Quad.cc src/Segment.cc src/TagDetection.cc src/TagDetector.cc src/TagFamily.cc src/UnionFindSimple.cc
)



#backcamera node
add_executable(backcamera_node
  ${ArmorDetectionProtoSrc}
  ${ArmorDetectionProtoHds}
  backcamera_node.cpp
  gimbal_control.cpp
  apriltag_detector.cpp
)

target_link_libraries(backcamera_node
  PRIVATE
  detection::tool_backcamera
  detection::constraint_set_backcamera
  apriltags
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
