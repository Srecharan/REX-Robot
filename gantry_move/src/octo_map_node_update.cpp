#include <iostream>
#include "ros/ros.h"
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

void pcl_octo_update(const octomap_msgs::OctomapConstPtr& msg){

    ros::NodeHandle nh;
    //ros::Publisher oct_pub = nh.advertise<octomap_msgs::Octomap>("/oct_msg",10);
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_point", 10);

    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    octomap::OcTree* OCT = dynamic_cast<OcTree*>(tree);

    cout<<"tree depth: "<<OCT->getTreeDepth()<<endl;

    unsigned int maxDepth = OCT->getTreeDepth();
    cout << "tree depth is " << maxDepth << endl;

    vector<OcTreeNode*> collapsed_occ_nodes;
    do {
      collapsed_occ_nodes.clear();
      for (OcTree::iterator it = OCT->begin(); it != OCT->end(); ++it)
      {
        if(OCT->isNodeOccupied(*it) && it.getDepth() < maxDepth)
        {
          collapsed_occ_nodes.push_back(&(*it));
        }
      }
      for (vector<OcTreeNode*>::iterator it = collapsed_occ_nodes.begin(); it != collapsed_occ_nodes.end(); ++it)
      {
        OCT->expandNode(*it);
      }
      cout << "expanded " << collapsed_occ_nodes.size() << " nodes" << endl;
    } while(collapsed_occ_nodes.size() > 0);

    vector<point3d> pcl;
    for (OcTree::iterator it = OCT->begin(); it != OCT->end(); ++it)
    {
      if(OCT->isNodeOccupied(*it))
      {
        pcl.push_back(it.getCoordinate());
      }
    }

    cout<<"pcl.size: "<<pcl.size()<<endl;


    /*

    pcl::PointCloud<PointXYZ>::Ptr input_cloud (new PointCloud<PointXYZ>);
    pcl::PointCloud<PointXYZ>::Ptr no_nan_cloud (new PointCloud<PointXYZ>);
    vector<int> no_nan_index;
    sensor_msgs::PointCloud2 input = *cloud;
    sensor_msgs::PointCloud2  output;

    pcl::fromROSMsg(input, *input_cloud);
    pcl::toROSMsg(*input_cloud, output);

    cout<<"RAW input Clous Size: "<<input_cloud->points.size()<<endl;
    pcl::removeNaNFromPointCloud(*input_cloud,*no_nan_cloud,no_nan_index);

    float res = 0.01;
    octomap::OcTree tree(res);

    for (size_t i = 0; i<no_nan_cloud->points.size(); i++){
        float x = no_nan_cloud->points[i].x;
        float y = no_nan_cloud->points[i].y;
        float z = no_nan_cloud->points[i].z;

        octomap::point3d coor(x,y,z);
        tree.updateNode(coor,true);
    }
    tree.updateInnerOccupancy();

    octomap_msgs::Octomap om;

    cout<<"publishing............."<<endl;

    ros::Rate loop_rate(1);

    while (ros::ok()){
        output.header.frame_id = "/map";
        output.header.stamp = ros::Time::now();
        pcl_pub.publish(output);

        om.binary = 1;
        om.id = 1;
        om.resolution = 0.05;
        om.header.frame_id = "/map";
        om.header.stamp = ros::Time::now();
        bool reso = octomap_msgs::fullMapToMsg(tree, om);
        oct_pub.publish(om);
        ros::spinOnce();
        loop_rate.sleep();
    }
    */
}

int main(int argc, char *argv[]){

    ros::init (argc, argv, "octomap_3");
    ROS_INFO("Started octomap_1 read node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<octomap_msgs::Octomap>("/oct_msg",1, pcl_octo_update);
    ros::spin();

    return 0;
}
