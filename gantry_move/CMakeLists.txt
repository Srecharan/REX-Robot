cmake_minimum_required(VERSION 2.8.3)
project(gantry_move)

#set(dir ${DYNAMIXEL_DIR} /usr/local/)

add_compile_options(-std=c++17)

find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

#find_package(VTK 6.3 REQUIRED)
#include(${VTK_USE_FILE})

find_package( OpenCV REQUIRED )
find_package(PCL 1.8 REQUIRED)

find_package(X11 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  #dynamixel_sdk
  actionlib
  actionlib_msgs
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
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  #gantry_msgs
  tf
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  cv_bridge
  tf_conversions
  eigen_conversions
)  

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Generate actions in the 'action' folder
 add_action_files(
   DIRECTORY action
   FILES gantry_move.action
 )

 generate_messages(
    DEPENDENCIES
    actionlib_msgs
    sensor_msgs
    std_msgs
    #gantry_msgs
 )

catkin_package(
INCLUDE_DIRS include
LIBRARIES gantry_move
CATKIN_DEPENDS actionlib eigen_conversions actionlib_msgs geometric_shapes image_transport interactive_markers message_filters message_generation moveit_core moveit_ros_perception moveit_ros_planning moveit_ros_planning_interface moveit_visual_tools pcl_conversions pcl_ros pluginlib rosbag roscpp rospy sensor_msgs std_msgs tf tf2 tf2_eigen tf2_geometry_msgs tf2_ros cv_bridge # gantry_msgs
)
# eigen
# ros_faster_rcnn
# dynamixel_sdk

#include_directories(${DYNAMIXEL_DIR}/include/dynamixel_sdk/)
#link_directories(${DYNAMIXEL_DIR}/lib)

#target_include_directories(mylib PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/mylib>
#  $<INSTALL_INTERFACE:include/mylib>  # <prefix>/include/mylib
#)

include_directories(  ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJECT_NAME}_server src/action_server.cpp)
add_executable(${PROJECT_NAME}_client src/action_client.cpp)
#add_executable(${PROJECT_NAME}_test src/just_import.cpp)
#add_executable(${PROJECT_NAME}_get_pose src/get_pose.cpp)
#add_executable(${PROJECT_NAME}_get_images src/get_images.cpp)
#add_executable(${PROJECT_NAME}_set_param src/pub_ros_param.cpp)
#add_executable(${PROJECT_NAME}_get_pose_new src/get_pose_new.cpp)
#add_executable(${PROJECT_NAME}_get_clicked src/clicked_point_subscriber.cpp)
#add_executable(${PROJECT_NAME}_clean_buds src/mega_buds_clean.cpp)
#add_executable(${PROJECT_NAME}_octo_node src/octo_map_node.cpp)





add_dependencies(${PROJECT_NAME}_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}_get_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}_get_images ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}_set_param ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}_get_pose_new ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}_get_clicked ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}_clean_buds ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME}_octo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_server ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_client ${catkin_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(${PROJECT_NAME}_get_pose ${catkin_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(${PROJECT_NAME}_get_images ${catkin_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(${PROJECT_NAME}_set_param ${catkin_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(${PROJECT_NAME}_get_pose_new ${catkin_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(${PROJECT_NAME}_get_clicked ${catkin_LIBRARIES} ${OpenCV_LIBS} ${X11_LIBRARIES})
#target_link_libraries(${PROJECT_NAME}_clean_buds ${catkin_LIBRARIES} ${OpenCV_LIBS} ${X11_LIBRARIES})
#target_link_libraries(${PROJECT_NAME}_octo_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${X11_LIBRARIES})
