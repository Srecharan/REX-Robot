cmake_minimum_required(VERSION 3.0.2)
project(gantry_driver)

add_compile_options(-std=c++11)

find_library(lsFoundation20 libsFoundation20.so /home/abhi/software_make/sFoundation/sFoundation)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  cv_bridge
  geometric_shapes
  image_transport
  interactive_markers
  message_filters
  message_generation
  moveit_core
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  pcl_conversions
  pcl_ros
  pluginlib
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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

# Generate actions in the 'action' folder
# add_action_files(
#   DIRECTORY action
#   FILES ServoMotorMulti.action

# )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs   control_msgs   sensor_msgs   std_msgs    #tf2_geometry_msgs
 )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES servo_controller_multi
#  CATKIN_DEPENDS actionlib actionlib_msgs control_msgs cv_bridge geometric_shapes image_transport interactive_markers message_filters message_generation moveit_core moveit_ros_perception moveit_ros_planning moveit_ros_planning_interface moveit_visual_tools pcl_conversions pcl_ros pluginlib roscpp rospy sensor_msgs std_msgs tf tf2_eigen tf2_geometry_msgs tf2_ros
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

#======================================================================================================
add_library(${PROJECT_NAME}_axis src/Axis.cpp)
add_library(${PROJECT_NAME}_supervisor src/Supervisor.cpp)
#======================================================================================================

add_executable(${PROJECT_NAME}_server src/server.cpp)

## Add cmake target dependencies of the library

add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_server)

## Specify libraries to link a library or executable target against

target_link_libraries(${PROJECT_NAME}_server ${catkin_LIBRARIES} ${lsFoundation20} ${PROJECT_NAME}_axis ${PROJECT_NAME}_supervisor)

