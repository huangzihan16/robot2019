cmake_minimum_required(VERSION 2.8.3)
project(initial_pose)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
# 添加cmake模块以使用g2o
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
  roborts_msgs
)
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )

catkin_package()

include_directories(
	${PROJECT_SOURCE_DIR}/include
  ${G2O_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR}
  "/usr/include/eigen3/"
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/initial_pose.cpp
# )

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/initial_pose_node.cpp)
#add_executable(initial_pose src/initial_pose.cpp)
#target_link_libraries(initial_pose ${CSPARSE_LIBRARY} g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension ${catkin_LIBRARIES})

#add_executable(initial_pose src/new_initial_pose.cpp)
#target_link_libraries(initial_pose
#	${CSPARSE_LIBRARY}
#	g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
#	${catkin_LIBRARIES})
	
add_executable(initial_pose_publisher
	src/likelihood_match.cpp
	src/initialpose_estimation.cpp
	src/initialpose_g2o.cpp
	src/initialpose_publish.cpp
)
target_link_libraries(initial_pose_publisher
	${CSPARSE_LIBRARY}
	g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
	${catkin_LIBRARIES}
)
add_dependencies(initial_pose_publisher roborts_msgs_generate_messages)