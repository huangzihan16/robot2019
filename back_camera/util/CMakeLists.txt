project(detection_util)

add_library(cv_toolbox_backcamera INTERFACE)
target_sources(cv_toolbox_backcamera INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(cv_toolbox_backcamera INTERFACE ${catkin_LIBRARIES}
  )
target_include_directories(cv_toolbox_backcamera
  INTERFACE
  ${catkin_INCLUDE_DIRS}
  )
add_library(detection::tool_backcamera ALIAS cv_toolbox_backcamera)
