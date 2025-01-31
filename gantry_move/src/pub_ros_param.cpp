#include <dirent.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <vector>

using namespace std;

int main(int argc, char **argv) {

  ros::init(argc, argv, "csv_publisher");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Int32>("/csv_done", 1000);

  std_msgs::Int32 a;
  a.data = 10;
  string s("/csv_written");
  int i;

  nh.setParam(s, -1);

  if (nh.getParam(s, i))
    ROS_INFO("SUCCESS..................");

  ros::spinOnce();
  ros::shutdown();

  return 0;
}
