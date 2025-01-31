#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/filter.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <deque>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

using namespace ros;
using namespace std;
using namespace pcl;
using namespace octomap;
//using namespace octree;

void pcl_octo(const sensor_msgs::PointCloud2ConstPtr& cloud)
{

    ros::NodeHandle nh;
    ros::Publisher oct_pub = nh.advertise<octomap_msgs::Octomap>("/oct_msg",5);
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_point", 5);

    pcl::PointCloud<PointXYZRGB>::Ptr input_cloud (new PointCloud<PointXYZRGB>);
    pcl::PointCloud<PointXYZRGB>::Ptr no_nan_cloud (new PointCloud<PointXYZRGB>);
    vector<int> no_nan_index;
    sensor_msgs::PointCloud2 input = *cloud;
    sensor_msgs::PointCloud2  output;

    pcl::fromROSMsg(input, *input_cloud);
    pcl::toROSMsg(*input_cloud, output);

    cout << "RAW input Clous Size: " << input_cloud->points.size() << endl;
    pcl::removeNaNFromPointCloud(*input_cloud, *no_nan_cloud, no_nan_index);

    float res = 0.025;
    octomap::OcTree tree(res);

    for (size_t i = 0; i < no_nan_cloud->points.size(); i++) {
        float x = no_nan_cloud->points[i].x;
        float y = no_nan_cloud->points[i].y;
        float z = no_nan_cloud->points[i].z;

        octomap::point3d coor(x, y, z);
        tree.updateNode(coor, true);
    }

    tree.updateInnerOccupancy();

    octomap_msgs::Octomap om;

    ros::Rate loop_rate(1);

    while (ros::ok()) {
        //sleep(1);
        cout << "publishing............." << endl;
        output.header.frame_id = "/camera_link";
        output.header.stamp = ros::Time::now();
        pcl_pub.publish(output);

        om.binary = 1;
        om.id = 1;
        om.resolution = res;
        om.header.frame_id = "/camera_link";
        om.header.stamp = ros::Time::now();
        bool reso = octomap_msgs::fullMapToMsg(tree, om);
        oct_pub.publish(om);
        ros::spinOnce();
        loop_rate.sleep();
    }

    cout << "while loop exited......." << endl;
}

int main(int argc, char* argv[])
{

    ros::init (argc, argv, "octomap_node");
    ROS_INFO("Started octomap_1 read node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_pcd", 1, pcl_octo);
    ros::spin();

    return 0;
}
