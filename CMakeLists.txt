cmake_minimum_required(VERSION 3.0.2)
project(swot_manipulation_bt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ur_ros_driver
  message_generation
  cartesian_control_msgs
  geometry_msgs
  pcl_ros
  swot_ur
  swot_msgs
  darknet_ros_msgs
  tf2_ros
  tf
  tf2
  sensor_msgs
  std_srvs
  std_msgs
  behaviortree_cpp
  actionlib_msgs
  actionlib
  control_msgs
  actionlib
  std_srvs
)

## The catkin_package macro generates cmake config files for the package
## Declare things to be passed to dependent projects
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES swot_manipulation_bt manipulation pick_classes place_classes shared_classes
 CATKIN_DEPENDS roscpp swot_ur swot_msgs behaviortree_cpp
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${swot_ur_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/src/swot_manipulation_bt/include
)

install(DIRECTORY include/swot_manipulation_bt/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#a must!
link_directories($ENV{HOME}/catkin_ws/devel/lib)

set(REQUIRED_FILES
  src/manipulation.cpp
  src/pick_classes.cpp
  src/place_classes.cpp
  src/shared_classes.cpp
)

add_library(${PROJECT_NAME}_lib
  ${REQUIRED_FILES}
)
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} behaviortree_cpp robotiq_2f_gripper ur_pass_through ur_rtde)

add_executable(swot_manipulation_bt src/main.cpp)
add_dependencies(swot_manipulation_bt ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})
target_link_libraries(swot_manipulation_bt ${catkin_LIBRARIES} behaviortree_cpp ur_rtde robotiq_2f_gripper ur_pass_through ${PROJECT_NAME}_lib)
