cmake_minimum_required(VERSION 2.8.3)
project(cam_input)

## Compile as C++11, supported in ROS Kinetic and newer
##add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)
include_directories(include ${OpenCV_INCLUDE_DIRS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(GENICAM_LIBRARY $ENV{GENICAM_ROOT_V2_3}/bin/Linux64_x64)
include_directories($ENV{GENICAM_ROOT_V2_3}/../../sdk/include  $ENV{GENICAM_ROOT_V2_3}/library/CPP/include )
AUX_SOURCE_DIRECTORY(src DIR_SRCS)
link_directories(${GENICAM_LIBRARY} ${GENICAM_LIBRARY}/../../../lib)

link_directories(/usr/local/lib)
include_directories(/usr/local/include/yaml-cpp)
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES video_transport_tutorial
#  CATKIN_DEPENDS cv_bridge image_transport robot_vison roscpp sensor_msgs stdmsgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



add_executable(cam
  src/cam.cpp
)

target_link_libraries(cam
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gxiapi 
  GenApi_gcc40_v2_3
  dximageproc
  log4cpp_gcc40_v2_3
  Log_gcc40_v2_3
  MathParser_gcc40_v2_3
  GCBase_gcc40_v2_3
  pthread
)
add_executable(cam_roi
  src/cam_roi.cpp
)

target_link_libraries(cam_roi
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gxiapi 
  GenApi_gcc40_v2_3
  dximageproc
  log4cpp_gcc40_v2_3
  Log_gcc40_v2_3
  MathParser_gcc40_v2_3
  GCBase_gcc40_v2_3
  pthread
)

add_executable(cam_test
  src/cam_test.cpp
)

target_link_libraries(cam_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gxiapi 
  GenApi_gcc40_v2_3
  dximageproc
  log4cpp_gcc40_v2_3
  Log_gcc40_v2_3
  MathParser_gcc40_v2_3
  GCBase_gcc40_v2_3
  pthread
  yaml-cpp
)



