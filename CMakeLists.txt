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
  behaviortree_cpp_v3
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
 LIBRARIES condition_classes swot_manipulation_bt manipulation pick_classes place_classes shared_classes
 CATKIN_DEPENDS roscpp swot_ur swot_msgs 
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${swot_ur_INCLUDE_DIRS}
  /home/irobot/catkin_ws/src/swot_manipulation_bt/include
)

install(DIRECTORY include/swot_manipulation_bt/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#a must!
link_directories($ENV{HOME}/catkin_ws/devel/lib)

##declaration of libraries
add_library(condition_classes src/condition_classes.cpp)
add_dependencies(condition_classes ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(condition_classes ${catkin_LIBRARIES} behaviortree_cpp_v3 robotiq_2f_gripper ur_pass_through ur_rtde)

add_library(manipulation src/manipulation.cpp)
add_dependencies(manipulation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manipulation ${catkin_LIBRARIES} behaviortree_cpp_v3 robotiq_2f_gripper ur_pass_through ur_rtde)

add_library(pick_classes src/pick_classes.cpp)
add_dependencies(pick_classes ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pick_classes ${catkin_LIBRARIES} behaviortree_cpp_v3 robotiq_2f_gripper ur_pass_through ur_rtde)

add_library(place_classes src/place_classes.cpp)
add_dependencies(place_classes ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(place_classes ${catkin_LIBRARIES} behaviortree_cpp_v3 robotiq_2f_gripper ur_pass_through ur_rtde)

add_library(shared_classes src/shared_classes.cpp)
add_dependencies(shared_classes ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(shared_classes ${catkin_LIBRARIES} behaviortree_cpp_v3 robotiq_2f_gripper ur_pass_through ur_rtde)

##declaration of executable files 
add_executable(swot_manipulation_bt src/main.cpp)
add_dependencies(swot_manipulation_bt ${${PROJECT_NAME}_EXPORTED_TARGETS}  ${catkin_EXPORTED_TARGETS})
target_link_libraries(swot_manipulation_bt condition_classes manipulation pick_classes place_classes shared_classes ${catkin_LIBRARIES} behaviortree_cpp_v3 robotiq_2f_gripper ur_pass_through ur_rtde)

