/* This code focuses on the tracking of the octree voxel content and finding connectivity between
 * two points using dijkstra search as a connectivity criteria
*/

#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <numeric>
#include <tuple>

#include <iostream>
#include <stdlib.h>
#include <sys/types.h>
#include <dirent.h>
#include <vector>
#include <string.h>
#include <string>
#include <fstream>
#include <math.h>
#include <time.h>
#include <unistd.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_container.h>
#include <pcl/octree/octree_nodes.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/segmentation/extract_clusters.h>

#include <eigen3/Eigen/Dense>
#include <boost/make_shared.hpp>
#include <boost/format.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>
#include <tf2/transform_datatypes.h>

#include<ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/cache.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace pcl;
using namespace sensor_msgs;
using namespace cv;
using namespace cv::ml;
using namespace std;


class octree_concepts{
public:

    //typedef pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGB>::FixedDepthIterator FDI;
    //typedef SampleConsensusModelLine<PointXYZ>::Ptr SampleConsensusModelLinePtr;
    //typedef vtkSmartPointer<vtkMutableUndirectedGraph> vtkmug;

    octree_concepts() {
        input_cloud = pcl::PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<PointXYZRGB>);
        no_nan_cloud = pcl::PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<PointXYZRGB>);
        cloudVoxel = pcl::PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<PointXYZRGB>);
        displayCloud = pcl::PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<PointXYZRGB>);
        final_cloud = pcl::PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<PointXYZRGB>);
        cane_cloud_final = pcl::PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<PointXYZRGB>);
        cordon_cloud_final = pcl::PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<PointXYZRGB>);
        temp_bud_cloud = pcl::PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<PointXYZRGB>);
        wires_fill = pcl::PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        this_cane = pcl::PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        cut_point_cloud_n = pcl::PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        cut_point_cloud_n_minus_1 = pcl::PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        only_cane = pcl::PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

        ransac_sample_cloud = pcl::PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);// (new pcl::PointCloud<pcl::PointXYZRGB>);
        final_ransac = pcl::PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        region_growing_stop_cloud = pcl::PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

        //pub_pose = nh.advertise<geometry_msgs::PoseArray>("/pose_",10);

        //svm = SVM::create();
        //svm = cv::Algorithm::load<ml::SVM>("/home/vision/QT/PCL_TEST/svm_model_1.xml");
        //read_pcd();
        //showCylinder();

        //pub_wire = nh.advertise<sensor_msgs::PointCloud2>("/wire",5,)
        
        MEGA_CLOUD = pcl::PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<PointXYZRGB>);
        BUD_CLOUD = pcl::PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<PointXYZRGB>);

        //mega_cloud.subscribe(nh, "/mega_cloud", 1, ros::TransportHints().unreliable().reliable().tcpNoDelay());
        //mega_buds.subscribe(nh,"/mega_buds", 1, ros::TransportHints().unreliable().reliable().tcpNoDelay());

        mega_cloud.subscribe(nh, "/mega_cloud", 1);
        mega_buds.subscribe(nh,"/mega_buds", 1);

        system("rm -r /home/agvbotics/ros/catkin_ws/src/rovin_move/src/FOLDER/*.pcd");
        system("rm -r /home/agvbotics/ros/catkin_ws/src/rovin_move/src/FOLDER/*.csv");

        //MySyncPolicy.setInterMessageLowerBound(1,ros::Duration{1});
        sync.reset(new Sync(MySyncPolicy(5), mega_cloud, mega_buds));
        //MySyncPolicy.setInterMessageLowerBound( .setAgePenalty(1); //3000
        //MySyncPolicy.setInterMessageLowerBound(0,ros::Duration{0.1});
        //sync->getPolicy()->setAgePenalty(1);
        //sync->getPolicy()->setInterMessageLowerBound(0, ros::Duration{0.2});
        //sync->getPolicy()->setMaxIntervalDuration(ros::Duration{0.3});
        //sync->getPolicy()->setMaxIntervalDuration(ros::Duration{1});
        sync->registerCallback(boost::bind(&octree_concepts::callback, this, _1, _2));

    }
            
    void callback(const  sensor_msgs::PointCloud2ConstPtr mega_cl, const  sensor_msgs::PointCloud2ConstPtr mega_bd){

        cout<<"Inside the get pose new subscriber..........."<<endl;

        ros::param::set("/terminate_loop_mega_cloud",-1);
        ros::param::set("/terminate_loop_mega_buds",-1 );

        sensor_msgs::PointCloud2 p1 = *mega_cl;
        sensor_msgs::PointCloud2 p2 = *mega_bd;

        p1.height = 1;
        p1.is_dense = false;
        p1.fields[3].name = "intensity";

        p2.height = 1;
        p2.is_dense = false;
        p2.fields[3].name = "intensity";

        pcl::fromROSMsg(p1, *MEGA_CLOUD);
        pcl::fromROSMsg(p2, *BUD_CLOUD);

        MEGA_CLOUD->is_dense = false;

        no_nan_index.clear();
        pcl::removeNaNFromPointCloud(*MEGA_CLOUD, *MEGA_CLOUD, no_nan_index);

        BUD_CLOUD->is_dense = false;
        no_nan_index.clear();

        pcl::removeNaNFromPointCloud(*BUD_CLOUD, *BUD_CLOUD, no_nan_index);

        cout<<"MEGA_CLOUD SIZE:"<<MEGA_CLOUD->points.size()<<endl;
        cout<<"BUD_CLOUD SIZE:"<<BUD_CLOUD->points.size()<<endl;

        no_nan_cloud->clear();

        #pragma omp parallel num_threads(10)
        for (size_t i = 0; i <MEGA_CLOUD->points.size(); i++){
            MEGA_CLOUD->points[i].r = 255;
            MEGA_CLOUD->points[i].g = 255;
            MEGA_CLOUD->points[i].b = 255;

            pcl::PointXYZRGB p;

            p.x = MEGA_CLOUD->points[i].x; p.y = MEGA_CLOUD->points[i].y; p.z = MEGA_CLOUD->points[i].z;
            p.r = MEGA_CLOUD->points[i].r; p.g = MEGA_CLOUD->points[i].g; p.b = MEGA_CLOUD->points[i].b;
            no_nan_cloud->points.push_back(p);
        }

        #pragma omp parallel num_threads(10)
        for (size_t i = 0; i <BUD_CLOUD->points.size(); i++){
            BUD_CLOUD->points[i].r = 255;
            BUD_CLOUD->points[i].g = 0;
            BUD_CLOUD->points[i].b = 0;

            pcl::PointXYZRGB p;

            p.x = BUD_CLOUD->points[i].x; p.y = BUD_CLOUD->points[i].y; p.z = BUD_CLOUD->points[i].z;
            p.r = BUD_CLOUD->points[i].r; p.g = BUD_CLOUD->points[i].g; p.b = BUD_CLOUD->points[i].b;
            no_nan_cloud->points.push_back(p);
        }


        pcl::io::savePCDFile("/home/agvbotics/ros/catkin_ws/src/rovin_move/src/FOLDER/mega_cloud.pcd",*MEGA_CLOUD,true);
        pcl::io::savePCDFile("/home/agvbotics/ros/catkin_ws/src/rovin_move/src/FOLDER/mega_buds.pcd",*BUD_CLOUD,true);


        ros::param::set("/image_captured", 1);
        sleep(1);

        system("/home/agvbotics/QT/build-point_clouds-Desktop-Default/pcl_svd_3_ros");

        ros::param::set("/cut_point_received", 1);
        ros::param::set("/terminate_loop_mega_buds",-1 );
        sleep(1);

        ROS_WARN("Pose written to CSV.......");


        }

protected:
    pcl::PointCloud<PointXYZRGB>::Ptr input_cloud;// (new PointCloud<PointXYZRGBRGB>);
    PointCloud<PointXYZRGB>::Ptr no_nan_cloud;// (new PointCloud<PointXYZRGBRGB>);
    vector<int> no_nan_index;
    //pcl::visualization::PCLVisualizer viz;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudVoxel;//(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr displayCloud;// (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud;// (new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr cordon_cloud_final; // (new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr cane_cloud_final;// (new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr temp_bud_cloud;// (new PointCloud<PointXYZRGB>);
    int depth;
    std::vector<int> bud_indices, line_inliers;
    //std::vector<string> pedg_id; // prdegree_id for graph
    //custom_svd SVD;
    //Ptr<SVM> svm;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr MEGA_CLOUD;// (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr BUD_CLOUD;// (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<int> good_ped_index, good_vertex_index_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr wires_fill;//( new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr this_cane;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cut_point_cloud_n, cut_point_cloud_n_minus_1;
    std::vector<int> cut_point_ind;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr only_cane;// (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<int> only_cane_index;
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree, kdtree_cutpoint;

    std::vector<int> indexVector;
    std::vector<std::vector<int>> indexVV;
    pcl::octree::OctreePointCloudPointVector<PointXYZRGB>::LeafNodeIterator it;

    std::vector< pcl::PointXYZRGB> pcl_stack;
    int done;
    int count_blue;
    float radius;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ransac_sample_cloud;// (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_ransac; // (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB searchPoint;

    int while_counter = 0;
    std::vector<int> inliers;

    cv::Mat a_cv;

    Eigen::Matrix3f C;
    Eigen::VectorXf a;

    pcl::PointXYZRGB p1 ,p2;
    Eigen::MatrixXf::Index maxRow, maxCol;//, minRow, minCol;
    Eigen::Matrix<float, Eigen::Dynamic, 3> coor;// (row, 3);
    std::vector<int> indices_good_buds;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr region_growing_stop_cloud;
    
    ros::Publisher pub;
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::PointCloud2> mega_cloud;
    message_filters::Subscriber<sensor_msgs::PointCloud2> mega_buds;
    typedef message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
    sensor_msgs::PointCloud2 output;
    sensor_msgs::PointCloud2 output_buds;
    ros::Publisher pub_bud;
    ros::Publisher pub_pose;
    ros::Publisher pub_cut;poses_
    ros::Publisher pub_wire, pub_cordon;
    geometry_msgs::Pose pose_msg;
    geometry_msgs::PoseArray poses_;
};

class fitted_lines{

public:
    fitted_lines(){}
    fitted_lines(vector<int> h, vector<int> v,Eigen::VectorXf hc,Eigen::VectorXf vc):horizontal_inliers(h),vertical_inliers(v),hc(hc),vc(vc){}
    vector<int> horizontal_inliers;
    vector<int> vertical_inliers;
    Eigen::VectorXf hc;
    Eigen::VectorXf vc;
};


int main(int argc, char **argv){
    ros::init(argc, argv, "cut_pose_estimator");
    ros::param::set("/csv_written","false");
    octree_concepts OC;
    ros::spin();
    return 0;
}

//int main(int argc, char** argv){
//    octree_concepts OC;
//    return 0;/
//}

