#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <numeric>
#include <tuple>
#include <stdlib.h>
#include <sys/types.h>
#include <dirent.h>
#include <fstream>
#include <math.h>
#include <time.h>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>

#include <X11/Xlib.h>
#include <iostream>
#include "X11/keysym.h"

using namespace std;
using namespace ros;


class clicked{
public:
    clicked() {
        count = 0;

        int res = system("rm -r /home/agvbotics/ros/catkin_ws/src/rovin_move/src/FOLDER/pose_clicked.csv");

        if (res > 0)
            ROS_WARN("Missing file.... not a problem... continue.... ");

        ROS_WARN("Press and hold ESC key while clicking a point to indicate the end of csv write function .... ");
        sub_ = nh_.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, &clicked::callback, this);
    }

    void callback(const geometry_msgs::PointStampedConstPtr& P){

        count++;
        geometry_msgs::PointStamped p = *P;
        cout<<"received point: "<<p.point.x <<", "<<p.point.y<<", "<<p.point.z<<"\t\t click count: "<<count<<endl;

        Display *dpy = XOpenDisplay(":0");
        char keys_return[32];
        XQueryKeymap(dpy, keys_return);

        cout<<"keys_return: "<<keys_return<<endl;
        KeyCode kc2 = XKeysymToKeycode(dpy, XK_Escape);

        bool isPressed = !!(keys_return[kc2 >> 3] & (1 << (kc2 & 7)));

        if (isPressed == true){
            cout<<"Escape button press detected......."<<endl;
            ros::param::set("/csv_written", -1);
            count = 0;
        }

        else{
            write_csv_vision(p.point.x, p.point.y, p.point.z, 0, 0, 0, 0);
            ros::param::set("/csv_written", "false");
        }
    }

    void write_csv_vision( float x_p, float y_p, float z_p, float x_o, float y_o, float z_o, int w_o){

        MyFile.open ("/home/agvbotics/ros/catkin_ws/src/rovin_move/src/FOLDER/pose_clicked.csv", ios::out | ios::app);
        MyFile <<x_p<<",";
        MyFile <<y_p<<",";
        MyFile <<z_p<<",";
        MyFile <<x_o<<",";
        MyFile <<y_o<<",";
        MyFile <<z_o<<",";
        MyFile <<w_o;
        MyFile <<"\n";
        MyFile.close();
    }
protected:

ros::NodeHandle nh_;
ros::Subscriber sub_;
ros::Publisher pub_;
geometry_msgs::PoseArray pose_arr_;
geometry_msgs::Pose pose_;
int count;
std::ofstream MyFile;

};

int main(int argc, char **argv){
    ros::init(argc, argv, "clicke_point_record");

    ros::param::set("/trigger_flash", "false");
    clicked ck;

    ros::spin();
    return 0;
}
