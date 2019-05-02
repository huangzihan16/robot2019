project(constraint_set_backcamera)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  ConstraintSetProtoSrc
  ConstraintSetProtoHds
  ${ProtoFiles}
  )

add_library(constraint_set_backcamera
  SHARED
  ${ConstraintSetProtoSrc}
  ${ConstraintSetProtoHds}
  constraint_set.cpp
  )

add_library(detection::constraint_set_backcamera ALIAS constraint_set_backcamera)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  detection::tool_backcamera
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  )
