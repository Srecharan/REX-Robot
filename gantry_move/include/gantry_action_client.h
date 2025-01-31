#ifndef GANTRY_ACTION_CLIENT_H
#define GANTRY_ACTION_CLIENT_H

#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/scoped_ptr.hpp>
#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <iterator>
#include <random>
#include <sstream>
#include <stdio.h>
#include <string>
#include <tuple>
#include <vector>

#include <memory>
#include <mutex>
#include <queue>
#include <string>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include "gantry_move/gantry_moveAction.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Transform.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define SCALE_FACTOR_INITIAL 0.1
#define SCALE_FACTOR_FINAL 0.1
#define SCALE_FACTOR_HALF 0.1
#define SCALE_FACTOR_CARTESIAN 0.1
#define SCALE_FACTOR_FULL 1.0
#define BACK_TRANSLATE 0.10 // 0.15 in meters
#define IMAGING_GAP 0.225
#define PI 3.14159265358979323846 /* pi */

using namespace std;
using namespace ros;
namespace rvt = rviz_visual_tools;
static const std::string PLANNING_GROUP = "manipulator";
static const std::string EE_GROUP = "endeffector";
bool success = false;
int ROBOT_MODE = 0; // 0 = semi auto; 1 = full auto

#endif // GANTRY_ACTION_CLIENT_H
