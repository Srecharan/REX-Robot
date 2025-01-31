#include <ros/ros.h>
#include <std_msgs/String.h>

void simpleCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_node");
  ros::NodeHandle nh;
  
  ros::Subscriber sub = nh.subscribe("chatter", 1000, simpleCallback);
  ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000);
  
  ros::Rate loop_rate(1); // 1 Hz
  
  while (ros::ok()) {
    std_msgs::String msg;
    msg.data = "Hello, ROS!";
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}

