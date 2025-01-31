#ifndef SVD_H
#define SVD_H

//===============CPP headers==========================
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <math.h>
#include <ctime>
#include <sstream>
#include <string>
#include "stdio.h"
#include "stdlib.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>


//===================PCL headers======================
#include <pcl/common/eigen.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/console/parse.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using namespace std;
class custom_svd{


//custom_svd(){}

public:
void compute_cov_matrix(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Matrix3f& C){


    Eigen::Vector4f xyz_centroid;
    pcl::compute3DCentroid (*cloud, xyz_centroid);
    //Eigen::Matrix3f C;
    pcl::computeCovarianceMatrix (*cloud, xyz_centroid, C);

    //return C;
}


void compute_SVD( Eigen::Matrix3f covariance_matrix, Eigen::VectorXf& res){

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariance_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    //vector<double> a;

    Eigen::VectorXf a(3);

    a<<svd.singularValues()[0],svd.singularValues()[1],svd.singularValues()[2];
    //cout<<"svd["<<0<<"]"<<svd.singularValues()[0]<<endl;
    //cout<<"svd["<<1<<"]"<<svd.singularValues()[1]<<endl;
    //cout<<"svd["<<2<<"]"<<svd.singularValues()[2]<<endl;

    res = a;
    //A.resize(3,1);
    //cout<<"A.size: "<<A.rows()<<" x "<<A.cols()<<endl;
    //A(0,1) = svd.singularValues()[0];
    //A(1,1) = svd.singularValues()[1];
    //A(2,1) = svd.singularValues()[2];
    //a.clear();
    //a.push_back(svd.singularValues()[0]);
    //a.push_back(svd.singularValues()[1]);
    //a.push_back(svd.singularValues()[2]);
    //cout<<a<<endl;

    //return a;
}

/*
void write_singular_values(){

    std::ofstream myfile("/home/anjana/catkin_ws/src/ro_slam/outputs/singular_values.csv",std::ofstream::app);
    myfile << a[0]<<","<<a[1]<<','<<a[2]<<endl;
    myfile.close();

}

*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_neighbors(int flag, int K,float radius,const pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr kdtree_ptr,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointXYZRGB searchPoint){

    //pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree = kdtree_ptr->;
    //kdtree.setInputCloud (cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sub_cloud ( new pcl::PointCloud<pcl::PointXYZRGB>);

    if (flag==0)

      { //k search
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
        if ( kdtree_ptr->nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            pcl::copyPointCloud(*cloud,pointIdxNKNSearch,*sub_cloud);
      }
    }


  else//Radius search
    {	std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
      if ( kdtree_ptr->radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
      { pcl::copyPointCloud(*cloud,pointIdxRadiusSearch,*sub_cloud);

      }

    }

    return sub_cloud;
}


//Eigen::Matrix3f C;
//std::vector<double> a;
};

#endif // SVD_H
