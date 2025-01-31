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

#include <boost/thread.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>
#include <random>

#include <boost/scoped_ptr.hpp>

#define SCALE_FACTOR_INITIAL 1.0
#define SCALE_FACTOR_FINAL 0.7
#define BACK_TRANSLATE 0.05 //5 cm

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

#include "/home/agvbotics/ros/catkin_ws/src/rovin_move/include/rovin_move/svd.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>

#define USER_MATRIX
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>
#include <vtkCleanPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkDataSetAttributes.h>
#include <vtkDoubleArray.h>
#include <vtkGraphLayoutView.h>
#include <vtkGraphAlgorithm.h>
#include <vtkIntArray.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkBoostConnectedComponents.h>
//#include <vtkMutableDirectedGraph.h>
#include <vtkEdgeListIterator.h>
#include <vtkVertexListIterator.h>
#include <vtkUndirectedGraph.h>
#include <vtkRenderedGraphRepresentation.h>
#include <vtkTextProperty.h>
#include <vtkIdTypeArray.h>
#include <vtkVariant.h>
#include <vtkVariantArray.h>
#include <vtkExtractSelectedGraph.h>
#include <vtkPointSource.h>
#include <vtkExtractSelection.h>
#include <vtkSelection.h>
#include <vtkSelectionNode.h>
#include <vtkPolyLine.h>
#include <vtkPolyData.h>
#include <vtkCylinderSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkMath.h>
#include <vtkSphereSource.h>
#include <vtkProperty.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkAutoInit.h>
#include <vtkGraph.h>

#include <vtkTree.h>

#include <vtkBoostPrimMinimumSpanningTree.h>
#include <vtkBoostBreadthFirstSearch.h>
#include <vtkDataSetAttributes.h>
#include <vtkTreeDFSIterator.h>
#include <vtkLookupTable.h>

VTK_MODULE_INIT(vtkRenderingOpenGL)
VTK_MODULE_INIT(vtkRenderingFreeType)
VTK_MODULE_INIT(vtkInteractionStyle)

#include <vtkDijkstraGraphGeodesicPath.h>
#include <vtkGraphToPolyData.h>

using namespace std;
using namespace pcl;
using namespace sensor_msgs;
using namespace cv;
using namespace cv::ml;
using namespace std;
using namespace octree;

pcl::octree::OctreePointCloudPointVector<pcl::PointXYZRGB>   OCTREE (0.01f);
pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGB> Octree (0.01f);
pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZRGB> OCtree (0.01f);
pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> oCtree (0.01f);

#define resolution 0.01f // 0.01
#define graph_res 0.01f
#define MIN_BUD_LOCATIONS 4
//bool wireframe = true;

//namespace rvt = rviz_visual_tools;
//static const std::string PLANNING_GROUP = "manipulator";
bool success = false;

//int count_call_back = 0;

class get_pose{

public:

    typedef SampleConsensusModelLine<PointXYZ>::Ptr SampleConsensusModelLinePtr;
    typedef vtkSmartPointer<vtkMutableUndirectedGraph> vtkmug;

    get_pose() {

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

        pub_pose = nh.advertise<geometry_msgs::PoseArray>("/pose_",10);
        pub_cut = nh.advertise<PointCloud2>("/cut_point_cloud",10);
        pub_wire = nh.advertise<PointCloud2>("/wire",10);
        //ros::param::set("/trigger_flash", "false");
        //ROS_WARN("Waiting for action server to start.");

        roll = 0.0; pitch = 0.0; yaw = 0.0;

        svm = SVM::create();
        svm = cv::Algorithm::load<ml::SVM>("/home/vision/QT/PCL_TEST/svm_model_1.xml");

        MEGA_CLOUD = pcl::PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<PointXYZRGB>);
        BUD_CLOUD = pcl::PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<PointXYZRGB>);

        clear_all_var();

        mega_cloud.subscribe(nh, "/mega_cloud", 1);
        mega_buds.subscribe(nh,"/mega_buds", 1);

        sync.reset(new Sync(MySyncPolicy(10), mega_cloud, mega_buds));
        sync->registerCallback(boost::bind(&get_pose::callback, this, _1, _2));
    }

    void clear_all_var(){
        //---------------> ALL CLOUDS
        input_cloud->clear();
        no_nan_cloud->clear();
        cloudVoxel->clear();
        displayCloud->clear();
        final_cloud->clear();
        cane_cloud_final->clear();
        cordon_cloud_final->clear();
        temp_bud_cloud->clear();
        wires_fill->clear();
        this_cane->clear();
        cut_point_cloud_n->clear();
        cut_point_cloud_n_minus_1->clear();
        only_cane->clear();
        ransac_sample_cloud->clear();
        final_ransac->clear();
        region_growing_stop_cloud->clear();
        MEGA_CLOUD->clear();
        BUD_CLOUD->clear();

        //----------------> ALL VECTORS

        only_cane_index.clear();
        cut_point_ind.clear();
        indexVector.clear();
        indexVV.clear();
        good_vertex_index_.clear();
        good_ped_index.clear();
        pcl_stack.clear();
        pointIdxRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();
        inliers.clear();
        bud_indices.clear();
        line_inliers.clear();
        pedg_id.clear(); // prdegree_id for graph
        no_nan_index.clear();
        indices_good_buds.clear();
        targetPose.clear();
        budPose.clear();
        good_region_index.clear();
        cluster_indices.clear();

        //----------------> OTHER VARIABLES

        roll = 0.0; pitch = 0.0; yaw = 0.0;
        it.reset();

        //----------------> OCTREES
        OCTREE.deleteTree(); OCTREE.setResolution(0.01f);
        Octree.deleteTree(); Octree.setResolution(0.01f);
        OCtree.deleteTree(); OCtree.setResolution(0.01f);
        oCtree.deleteTree(); oCtree.setResolution(0.01f);
    }

    void callback(const  sensor_msgs::PointCloud2ConstPtr mega_cl, const  sensor_msgs::PointCloud2ConstPtr mega_bd){

        clear_all_var();
        cout<<"Inside the pcl mega buds subscriber..........."<<endl;

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

        pub_bud = nh.advertise<sensor_msgs::PointCloud2>("buds", 1000);

        cout<<"publishing buds. begins  ...."<<endl;

        pcl::toROSMsg(*BUD_CLOUD, output_buds);

    //for (int i = 0; i <3; i++){
        output_buds.header.frame_id = "camera_link";
        output_buds.header.stamp = ros::Time::now();
        pub_bud.publish(output_buds);
        //sleep(1);

        cout<<"publishing bud done......................."<<endl;
    //}

        svd_compute();

        setup_target();

        set_cut_point_orientation();

        sleep(1);

        cout<<"clearing variables ........"<<endl;
        clear_all_var();

        ros::param::set("/terminate_loop_cut_point",-1);
    }

    void read_pcd(){

        io::loadPCDFile("/media/HDD/data/Vineyard/2019_04_13_CLEREL_DORMANT_ROBOT/4_13_2019_R17_V1/extracted/bag_0/pcd2/stitched.pcd",*input_cloud);

        if (input_cloud->points.size() ==0 ){
            cout<<"Error loading PCD file input....."<<endl;
            exit(1);
        }

        #pragma omp parallel num_threads(10)
        for (size_t i = 0; i <input_cloud->points.size(); i++){
            input_cloud->points[i].r = 255;
            input_cloud->points[i].g = 255;
            input_cloud->points[i].b = 255;
        }

        //cout<<"RAW input Cloud Size: "<<input_cloud->points.size()<<endl;
        pcl::removeNaNFromPointCloud(*input_cloud,*no_nan_cloud,no_nan_index);

        //temp_bud();
        io::loadPCDFile("/media/HDD/data/Vineyard/2019_04_13_CLEREL_DORMANT_ROBOT/4_13_2019_R17_V1/extracted/bag_0/pcd2/buds.pcd",*temp_bud_cloud);

        if (temp_bud_cloud->points.size() ==0 ){
            cout<<"Error loading PCD file bud ....."<<endl;
            exit(1);
        }

        #pragma omp parallel num_threads(10)
        for (size_t i = 0; i <temp_bud_cloud->points.size(); i++){
            temp_bud_cloud->points[i].r = 255;
            temp_bud_cloud->points[i].g = 0;
            temp_bud_cloud->points[i].b = 0;
        }

        *no_nan_cloud = *no_nan_cloud + *temp_bud_cloud;
        svd_compute();

        // showCubes(std::sqrt (OCTREE.getVoxelSquaredSideLen (OCTREE.getTreeDepth())), cloudVoxel);
        // display_pcl(no_nan_cloud);
    }

    void svd_compute(){

        clock_t begin = clock();

        //int counter = 0;

        cout<<"no_nan_cloud...size: "<<no_nan_cloud->points.size()<<endl;

        OCTREE.deleteTree();
        OCTREE.setInputCloud(no_nan_cloud); // to get the content of the octree voxel OCTREE is OctreePointCloudPointVector type
        OCTREE.defineBoundingBox();
        OCTREE.addPointsFromInputCloud();
        it.reset();

        indexVector.clear();
        indexVV.clear();
        pcl_stack.clear();
        done = 0;
        count_blue = 0;
        radius = 0.05f;

        kdtree.setInputCloud (no_nan_cloud);

        pointIdxRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();
        while_counter = 0;
        inliers.clear();
        a_cv.empty();

        ransac_sample_cloud->clear();
        final_ransac->clear();
        region_growing_stop_cloud->clear();

        cut_point_cloud_n->clear();
        cut_point_cloud_n_minus_1->clear();

        for(it = OCTREE.leaf_begin(); it!=OCTREE.leaf_end(); ++it){

            //counter++;
            indexVector.clear();
            const pcl::octree::OctreePointCloud<PointXYZRGB>::LeafNode *node =  static_cast<const pcl::octree::OctreePointCloud<PointXYZRGB>::LeafNode*> (it.getCurrentOctreeNode());
            node->getContainer().getPointIndices(indexVector);// this contains the points within a voxel of the octree
            indexVV.push_back(indexVector);                   // this is the vector of a vector that contains all the points in PCL per voxel

            #pragma omp parallel num_threads(10)
            for (size_t i = 0; i< indexVector.size();i++ ){

                if (no_nan_cloud->points[indexVector[i]].r >=250 && no_nan_cloud->points[indexVector[i]].g< 20 && no_nan_cloud->points[indexVector[i]].b<20){
                    //cout<<indexVector[i]<<" ";
                    //cout<<no_nan_cloud->points[indexVector[i]]<<endl;
                    bud_indices.push_back(indexVector[i]);
                }
            }
        }

        for(size_t ii =0 ; ii < bud_indices.size(); ii++){

            if(no_nan_cloud->points[bud_indices[ii]].r==0 && no_nan_cloud->points[bud_indices[ii]].g==0 && no_nan_cloud->points[bud_indices[ii]].b==255){
                //cout<<"Bud Location already visited"<<endl;
                indices_good_buds.push_back(bud_indices[ii]);
                continue;
            }

            else{

                pcl_stack.clear();
                done = 0;
                count_blue = 0;
                while_counter = 0;
                pcl_stack.push_back(no_nan_cloud->points[bud_indices[ii]]);

                while(done == 0){

                    searchPoint = pcl_stack[0];

                    // ====================================== Nearest radial Search ==========================

                    pointIdxRadiusSearch.clear();
                    pointRadiusSquaredDistance.clear();
                    kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

                    if(  pointIdxRadiusSearch.size() >= 10 ){ // this handles if the radial search has very few neighbours and prevents ransac from failing

                        ransac_sample_cloud->empty();
                        final_ransac->empty();

                        #pragma omp parallel num_threads(3)
                        for (size_t j = 0; j < pointIdxRadiusSearch.size (); j++){
                            if (no_nan_cloud->points[pointIdxRadiusSearch[j]].r == 0 &&
                                no_nan_cloud->points[pointIdxRadiusSearch[j]].g == 0 &&
                                no_nan_cloud->points[pointIdxRadiusSearch[j]].b == 255){
                                count_blue++;
                            }
                        }

                        count_blue = 0;

                        if (count_blue == (int)pointIdxRadiusSearch.size()){
                            pcl_stack.erase(pcl_stack.begin());
                            continue;
                        }
                        else{

                            pcl::copyPointCloud<pcl::PointXYZRGB>(*no_nan_cloud, pointIdxRadiusSearch, *ransac_sample_cloud);

                            SVD.compute_cov_matrix(ransac_sample_cloud, C);
                            SVD.compute_SVD(C,a);

                            a_cv = (cv::Mat_<float>(1,3)<<a(0),a(1),a(2));

                            if(svm->predict(a_cv) > 0){

                                indices_good_buds.push_back(bud_indices[ii]);

                                #pragma omp parallel num_threads(3)
                                for (size_t j = 0; j < pointIdxRadiusSearch.size (); j++){

                                    no_nan_cloud->points[pointIdxRadiusSearch[j]].r = 0;//color_val.x;
                                    no_nan_cloud->points[pointIdxRadiusSearch[j]].g = 0;//color_val.y;
                                    no_nan_cloud->points[pointIdxRadiusSearch[j]].b = 255;//color_val.z;
                                }

                                inliers.clear();
                                pcl::SampleConsensusModelLine<pcl::PointXYZRGB>::Ptr  model_l (new pcl::SampleConsensusModelLine<pcl::PointXYZRGB> (ransac_sample_cloud));
                                pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_l,0.025);
                                ransac.computeModel ();
                                ransac.getInliers(inliers);
                                final_ransac->clear();

                                pcl::copyPointCloud<pcl::PointXYZRGB>(*ransac_sample_cloud, inliers, *final_ransac);
                                coor.resize(final_ransac->points.size (), 3);

                                #pragma omp parallel num_threads(3)
                                for (size_t k = 0; k < final_ransac->points.size (); k++){
                                    coor(k,0) = final_ransac->points[k].x;
                                    coor(k,1) = final_ransac->points[k].y;
                                    coor(k,2) = final_ransac->points[k].z;
                                }

                                Eigen::MatrixXf _sq = squareform(coor);
                                _sq.maxCoeff(&maxRow, &maxCol);

                                final_ransac->points[maxRow].r = 0; final_ransac->points[maxRow].g = 255; final_ransac->points[maxRow].b = 0;
                                final_ransac->points[maxCol].r = 0; final_ransac->points[maxCol].g = 255; final_ransac->points[maxCol].b = 0;

                                //string s1 = "ransa_cloud" + to_string(counter);
                                //string s2 = "ransac_final_cloud" + to_string(counter);
                                //string s3 = "seed_cloud" +  to_string(counter);

                                p1.x = coor(maxCol,0),  p1.y = coor(maxCol,1), p1.z = coor(maxCol,2);
                                p2.x = coor(maxRow,0),  p2.y = coor(maxRow,1), p2.z = coor(maxRow,2);

                                if (is_all_blue(p1, kdtree, radius) == false)
                                    pcl_stack.push_back(p1);
                                if (is_all_blue(p2, kdtree, radius) == false)
                                    pcl_stack.push_back(p2);

                                pcl::PointXYZRGB last_val = pcl_stack[0];

                                if (pcl_stack.size()!=0)
                                    pcl_stack.erase(pcl_stack.begin());

                                if(int(pcl_stack.size()) == 0){
                                    done = 1;

                                    if(last_val.r < 10 && last_val.g < 10 && last_val.b <10){
                                        //cout<<"from here....."<<endl;
                                        pcl::PointXYZRGB A;
                                        A.x = last_val.x;
                                        A.y = last_val.y;
                                        A.z = last_val.z;
                                        A.r = 255;
                                        A.g = 255;
                                        A.b = 0;
                                        //p1.r = 255; p1.g = 255; p1.b = 0;
                                        //if(is_all_blue_few_red(searchPoint,kdtree, 0.015) == false) //A
                                            region_growing_stop_cloud->points.push_back(A);
                                    }

                                    //indices_region_grow_stop.push_back(no_nan_cloud->points[]);
                                }
                                else
                                    done = 0;
                                //viz.addPointCloud<pcl::PointXYZRGB>(no_nan_cloud, to_string(rand()) + "cloud");
                                //viz.addPointCloud<pcl::PointXYZRGB>(final_ransac, s2.c_str());
                                //viz.addPointCloud<pcl::PointXYZRGB>(seed_cloud, s3.c_str());
                                //viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,4.0,s3.c_str());
                                //viz.spinOnce();
                                //counter++;
                               }

                            else{
                                done = 1;
                                //cout<<"color: "<<searchPoint<<endl;
                                if(searchPoint.r < 10 && searchPoint.g < 10 && searchPoint.b <10){

                                    pcl::PointXYZRGB A;
                                    A.x = searchPoint.x;
                                    A.y = searchPoint.y;
                                    A.z = searchPoint.z;
                                    A.r = 255;
                                    A.g = 255;
                                    A.b = 0;
                                    //p1.r = 255; p1.g = 255; p1.b = 0;

                                    //if(is_all_blue_few_red(searchPoint,kdtree, 0.015) == false) //A
                                        region_growing_stop_cloud->points.push_back(A);
                                }
                            }
                        }
                    }
                    else{

                        #pragma omp parallel
                        for (size_t j = 0; j < pointIdxRadiusSearch.size (); j++){
                            no_nan_cloud->points[pointIdxRadiusSearch[j]].r = 0;
                            no_nan_cloud->points[pointIdxRadiusSearch[j]].g = 0;
                            no_nan_cloud->points[pointIdxRadiusSearch[j]].b = 255;
                            pcl_stack.erase(pcl_stack.begin());
                            done = 1;
                        }
                    }
                    while_counter++;
                }
                while_counter = 0;
                //string S = "segment" + to_string(ii);
                //viz.addPointCloud<pcl::PointXYZRGB>(no_nan_cloud, S.c_str());
                //viz.spinOnce();
            }
        }


        //cout<<"bud cloud size before: "<<indices_good_buds.size()<<endl;
        std::sort(indices_good_buds.begin(),indices_good_buds.end());
        indices_good_buds.erase(std::unique(indices_good_buds.begin(), indices_good_buds.end()), indices_good_buds.end());

        //cout<<"bud cloud size: "<<temp_bud_cloud->points.size()<<endl;
        //cout<<"good_bud_size after: "<<indices_good_buds.size()<<endl;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr good_bud_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

        #pragma omp parallel for
        for(std::vector<int>::iterator it = indices_good_buds.begin(); it!= indices_good_buds.end(); ++it){
            //cout<<"good_bud: "<<*it<<endl;
            p1.x = no_nan_cloud->points[*it].x;
            p1.y = no_nan_cloud->points[*it].y;
            p1.z = no_nan_cloud->points[*it].z;

            p1.r = 255; p1.g = 0; p1.b = 0;
            good_bud_cloud->points.push_back(p1);

            no_nan_cloud->points[*it].r = 255;
            no_nan_cloud->points[*it].g = 0;
            no_nan_cloud->points[*it].b = 0;

         }

        ///cout<<"region cloud size before: "<< region_growing_stop_cloud->points.size()<<endl;

        eudlidean_cluster(region_growing_stop_cloud);

        //cout<<"region cloud size after : "<< region_growing_stop_cloud->points.size()<<endl;

        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        cout<<"time elapsed: "<<elapsed_secs<<endl;

        fit3Dlines(0.01f);

        //========== removing buds near trellis wire ====================

        if (wires_fill->points.size()!=0){
            for(size_t t = 0; t <wires_fill->points.size(); t++ ){

                searchPoint = wires_fill->points[t];

                pointIdxRadiusSearch.clear();
                pointRadiusSquaredDistance.clear();
                kdtree.radiusSearch (searchPoint, 0.075f, pointIdxRadiusSearch, pointRadiusSquaredDistance);

                if(  pointIdxRadiusSearch.size() >= 3 ){

                    #pragma omp parallel num_threads(3)
                    for (size_t j = 0; j < pointIdxRadiusSearch.size (); j++){
                        if (no_nan_cloud->points[pointIdxRadiusSearch[j]].r >= 250 && // looking for red
                            no_nan_cloud->points[pointIdxRadiusSearch[j]].g <= 10 &&
                            no_nan_cloud->points[pointIdxRadiusSearch[j]].b <= 10){

                            no_nan_cloud->points[pointIdxRadiusSearch[j]].r = 0;
                            no_nan_cloud->points[pointIdxRadiusSearch[j]].g = 0;
                            no_nan_cloud->points[pointIdxRadiusSearch[j]].b = 255;
                        }
                    }
                }
            }
        }

        pcl::toROSMsg(*wires_fill, output_buds);

        //for (int i = 0; i <3; i++){
            output_buds.header.frame_id = "camera_link";
            output_buds.header.stamp = ros::Time::now();
            pub_wire.publish(output_buds);
            //sleep(1);

            cout<<"publishing wire done......................."<<endl;
        //}

        // ============================== working on the regison stop cloud to search and associate buds to canes ==================

        //vector<int>good_region_index; // outlier index

        good_region_index.clear();
        for(int it = 0; it < region_growing_stop_cloud->points.size(); it++){

            if (is_all_blue_few_red(region_growing_stop_cloud->points[it],kdtree, 0.015) == false){

                //cout<<"Index: "<<it<<endl;
                good_region_index.push_back(it);

            }
            //cout<<"Region Cloud["<<it<<"]: "<<region_growing_stop_cloud->points[it]<<endl;
        }
        pcl::copyPointCloud(*region_growing_stop_cloud,good_region_index,*region_growing_stop_cloud); // copy and remove outlier index values

        //viz.addPointCloud<pcl::PointXYZRGB>(no_nan_cloud, "final_result");
        //viz.addPointCloud<pcl::PointXYZRGB>(region_growing_stop_cloud, "final_disconnect");
        //viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,6,"final_disconnect");
        //viz.addPointCloud<pcl::PointXYZRGB>(wires_fill, "wires");
        //viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,8,"wires");
        //viz.spin();

        // ==================== cylinderical search ===============
        // ======================= seperate blue and  white Points ==================
        // start accumulating the blue cliud index from top

        // ==== for now brute force =============
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr only_cane (new pcl::PointCloud<pcl::PointXYZRGB>);
        //std::vector<int> only_cane_index;

        only_cane->clear();
        only_cane_index.clear();
        for(int it_ = 0; it_ < no_nan_cloud->points.size(); it_++){
            if (  no_nan_cloud->points[it_].r == 255     // 255
                  && no_nan_cloud->points[it_].g == 255     // 0
                  && no_nan_cloud->points[it_].b == 255
                  //|| (no_nan_cloud->points[pointIdxRadiusSearch[it]].r == 255     // 255
                  //    && no_nan_cloud->points[pointIdxRadiusSearch[it]].g == 255     // 0
                  //    && no_nan_cloud->points[pointIdxRadiusSearch[it]].b == 0)
                  ){

                //only_cane_index.push_back(it);

            }
            else
                only_cane_index.push_back(it_);
            //cout<<"Region Cloud["<<it<<"]: "<<region_growing_stop_cloud->points[it]<<endl;
        }

        pcl::copyPointCloud(*no_nan_cloud, only_cane_index, *only_cane);
        *only_cane = *only_cane + *region_growing_stop_cloud;

        // =============== extract cane clusters =====================

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr ec_tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
        ec_tree->setInputCloud (only_cane);


        cluster_indices.clear();
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (25);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (ec_tree);
        ec.setInputCloud (only_cane);
        ec.extract (cluster_indices);

        cout<<"cluster_indices: "<<cluster_indices.size()<<endl;

        cut_point_ind.clear();
        cut_point_cloud_n->clear();
        cut_point_cloud_n_minus_1->clear();

        for (std::vector<pcl::PointIndices>::const_iterator itt = cluster_indices.begin (); itt != cluster_indices.end (); ++itt){
            this_cane->clear();
            cloudVoxel->clear();
            good_ped_index.clear();
            good_vertex_index_.clear();
            cloudVoxel->clear();
            pedg_id.clear();

            for (std::vector<int>::const_iterator pit = itt->indices.begin (); pit != itt->indices.end (); ++pit)
                this_cane->points.push_back (only_cane->points[*pit]); //*
            //======= creating graph ==================

            OCTREE.deleteTree();
            OCTREE.setResolution(0.03f);
            OCTREE.setInputCloud(this_cane); // to get the content of the octree voxel OCTREE is OctreePointCloudPointVector type
            OCTREE.defineBoundingBox();
            OCTREE.addPointsFromInputCloud();

            //pcl::octree::OctreePointCloudPointVector<PointXYZRGB>::LeafNodeIterator it;

            cout<<"OCTREE.leaf count:"<<OCTREE.getLeafCount()<<endl;

            it.reset();
            int root_ped_id;

            indexVector.clear();
            indexVV.clear();

            for(it = OCTREE.leaf_begin(); it!=OCTREE.leaf_end(); ++it){

                pcl::PointXYZRGB pt_voxel_center;
                Eigen::Vector3f voxel_min, voxel_max;
                OCTREE.getVoxelBounds(it, voxel_min, voxel_max);

                pt_voxel_center.x = (voxel_min.x () + voxel_max.x ()) / 2.0f;
                pt_voxel_center.y = (voxel_min.y () + voxel_max.y ()) / 2.0f;
                pt_voxel_center.z = (voxel_min.z () + voxel_max.z ()) / 2.0f;

                pt_voxel_center.r = 0;
                pt_voxel_center.g = 255;
                pt_voxel_center.b = 255;

                //cout<<"index vector: size: "<<indexVector.size()<<endl;
                const pcl::octree::OctreePointCloud<PointXYZRGB>::LeafNode *node =  static_cast<const pcl::octree::OctreePointCloud<PointXYZRGB>::LeafNode*> (it.getCurrentOctreeNode());
                node->getContainer().getPointIndices(indexVector);// this contains the points within a voxel of the octree
                std::string s = to_string(node->getContainer().getPointIndex()); // we don't need the content of the vexel as the octree class assigns unique vixel index to each voxel
                pedg_id.push_back(s);

                for (size_t i = 0; i< indexVector.size();i++ ){

                    /*
                    if(this_cane->points[indexVector[i]].r >= 250
                            && this_cane->points[indexVector[i]].g >=250
                            && this_cane->points[indexVector[i]].b <= 10){
                        //good_ped_index.push_back(counter);
                        good_vertex_index_.push_back(node->getContainer().getPointIndex());
                        pt_voxel_center.r = 255;  pt_voxel_center.g = 255;  pt_voxel_center.b = 0;
                        root_ped_id = node->getContainer().getPointIndex();
                        cout<<"root set to: "<<root_ped_id<<endl;
                        break;
                    }
                    */

                    if(this_cane->points[indexVector[i]].r >= 250
                            //&& this_cane->points[indexVector[i]].g <= 10
                            //&& this_cane->points[indexVector[i]].b <= 10){
                        //good_ped_index.push_back(counter);
                            ){
                        good_vertex_index_.push_back(node->getContainer().getPointIndex());
                        pt_voxel_center.r = 255;  pt_voxel_center.g = 0;  pt_voxel_center.b = 0;
                        break;
                    }
                }
                cloudVoxel->points.push_back (pt_voxel_center); // this contains the centorid of the vexel, voxel defined as the white box
            }

            //string x_ = to_string(rand()) + "octree voxel";
            //viz.addPointCloud<pcl::PointXYZRGB>(cloudVoxel, x_);
            //viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,4.0,x_);

            //pcl::removeNaNFromPointCloud(*cloudVoxel, *cloudVoxel,no_nan_index);
            //pcl::io::savePCDFile("/media/HDD/data/Vineyard/ideal_lab/temp/small_pcd/cloud_voxewl.pcd", *cloudVoxel, true);

            //showCubes(std::sqrt (OCTREE.getVoxelSquaredSideLen (OCTREE.getTreeDepth())), cloudVoxel);
            //viz.addPointCloud<pcl::PointXYZRGB>(only_cane,to_string(rand())+"octree");
            //viz.spin();

            cout<<"good_vertex_index_.size(): "<<good_vertex_index_.size()<<endl;

            for(int a = 0 ; a<good_vertex_index_.size(); a++){

                cout<<"good_vertex_index_["<<a<<"] "<<good_vertex_index_[a]<< "-> "<< this_cane->points[good_vertex_index_[a]]<<endl;
                if(   this_cane->points[good_vertex_index_[a]].r >= 250
                   && this_cane->points[good_vertex_index_[a]].g >= 250
                   && this_cane->points[good_vertex_index_[a]].b <= 50
                        )
                    root_ped_id = good_vertex_index_[a];
            }

            cout<<"root ped estimate: "<<root_ped_id<<endl;

            if (root_ped_id >=0 && good_vertex_index_.size() >MIN_BUD_LOCATIONS ){   // only is we can find yellow dot and the bud count is > 2

                vtkmug g = vtkmug::New();
                g = generate_graph(*cloudVoxel,0.03);

                //display_graph(g);

                return_max_component_graph(g);

                //display_graph(g);

                // ====================== DFS tree search =========================

                //cout<<"no. of vertices: "<<g->GetNumberOfVertices()<<endl;

                vtkSmartPointer<vtkVertexListIterator> v_it  = vtkSmartPointer<vtkVertexListIterator>::New();
                g->GetVertices(v_it);

                vtkSmartPointer<vtkDoubleArray> weights = vtkSmartPointer<vtkDoubleArray>::New();
                weights->SetNumberOfComponents(1);
                weights->SetName("Weights");

                //====== getting the simplified vertives and edges of the graph
                std::vector<int> simplfied_graph_vextices;

                while(v_it->HasNext()){
                    vtkIdType v = v_it->Next();
                    //cout<<"remaining vertices: ["<<v<<"]"<< g->GetVertexData()->GetPedigreeIds()->GetVariantValue(v).ToString()<<endl;
                    //weights->InsertNextValue(1000.0);
                    simplfied_graph_vextices.push_back(g->GetVertexData()->GetPedigreeIds()->GetVariantValue(v).ToInt());
                }

                vtkSmartPointer<vtkEdgeListIterator> e_it  = vtkSmartPointer<vtkEdgeListIterator>::New();

                g->GetEdges(e_it);
                while(e_it->HasNext()){
                    e_it->Next();
                    weights->InsertNextValue(1000.0);
                }

                //for(int a = 0 ; a<simplfied_graph_vextices.size(); a++)
                //    cout<<"simplified: "<<simplfied_graph_vextices[a]<<endl;

                g->GetEdgeData()->AddArray(weights);
                // ====================== assigining root node to the graph =============
                std::vector<int>::iterator it_vec = std::find(simplfied_graph_vextices.begin(), simplfied_graph_vextices.end(), root_ped_id);
                int index_it_vec = std::distance(simplfied_graph_vextices.begin(), it_vec);

                cout<<"index_it_vec: "<<index_it_vec<<endl;
                //cout<<"simplfied_graph_vextices: "<<simplfied_graph_vextices.size()<<endl;

                if (index_it_vec >= simplfied_graph_vextices.size())
                    index_it_vec = index_it_vec - 1;

                //cout<<"index_it_vec: "<<index_it_vec<<endl;  // later be sure this has  value as a cross check

                // Output original graph info
                //std::cout << "Original Graph" << std::endl << "----------" << std::endl;
                //std::cout << "Number of vertices: "
                //          << g->GetNumberOfVertices() << std::endl;
                //std::cout << "Number of edges: "
                //          << g->GetNumberOfEdges() << std::endl;

                vtkSmartPointer<vtkBoostPrimMinimumSpanningTree> minimumSpanningTreeFilter = vtkSmartPointer<vtkBoostPrimMinimumSpanningTree>::New();
                minimumSpanningTreeFilter->SetOriginVertex(index_it_vec); // this value is yellow point on the octree
                minimumSpanningTreeFilter->SetInputData(g);
                minimumSpanningTreeFilter->SetEdgeWeightArrayName("Weights");
                minimumSpanningTreeFilter->Update();

                vtkSmartPointer<vtkTree> minimumSpanningTree = vtkSmartPointer<vtkTree>::New();
                minimumSpanningTree->ShallowCopy(minimumSpanningTreeFilter->GetOutput());
               // minimumSpanningTree->DeepCopy(minimumSpanningTreeFilter->GetOutput());

                vtkIdType root = minimumSpanningTree->GetRoot();
                cout<<"root node min span: "<<root<<endl;

                vtkSmartPointer<vtkTreeDFSIterator> dFS = vtkSmartPointer<vtkTreeDFSIterator>::New();
                dFS->SetStartVertex(root);// root
                dFS->SetTree(minimumSpanningTree);

                // the vertex id here are the indices of the original graph

                std::vector<int> dFS_output;
                while(dFS->HasNext()){

                    //cout<<"root vertex = "<<dFS->GetStartVertex()<<endl;
                  vtkIdType NextVertex = dFS->Next();
                  dFS_output.push_back(g->GetVertexData()->GetPedigreeIds()->GetVariantValue(NextVertex).ToInt());

                }
                //minimumSpanningTree->Dump();

                //for(int ii = 0; ii <dFS_output.size(); ii++)
                //    cout<<" dFS_output: "<<dFS_output[ii]<<endl;


                // ========================== Rearrange the bud sequence ======================
                // compare each element of dFS output to bud locations
                std::vector<int> final_bud_seq_index;

                for (int b_ = 0 ; b_ <dFS_output.size(); b_++){

                    int temp_index = dFS_output[b_];
                    //std::find(good_vertex_index_.begin(), good_vertex_index_.end(), temp_index);
                    std::vector<int>::iterator it_check = std::find(good_vertex_index_.begin(), good_vertex_index_.end(), temp_index);
                    int index_it_check = std::distance(good_vertex_index_.begin(), it_check);

                    if(index_it_check<good_vertex_index_.size())
                        final_bud_seq_index.push_back(good_vertex_index_[index_it_check]);
                        //cout<<temp_index<<"   "<<dFS_output[b_]<<endl;

                    //}
                    //cout<<"------------------------------------"<<endl;
                }

                //cout<<"final_bud_seq_index: "<<final_bud_seq_index.size()<<endl;

                for(size_t ll = 0; ll < final_bud_seq_index.size(); ll++){
                    //cout<<final_bud_seq_index[ll]<<" -> ";
                    cout<<this_cane->points[final_bud_seq_index[ll]]<<endl;
                }

                //cut_point_ind.push_back(final_bud_seq_index[MIN_BUD_LOCATIONS]);
                cut_point_cloud_n->points.push_back(this_cane->points[final_bud_seq_index[MIN_BUD_LOCATIONS]]);  //good_vertex_index_[
                cut_point_cloud_n_minus_1->points.push_back(this_cane->points[final_bud_seq_index[MIN_BUD_LOCATIONS-1]]);
                //cout<<"cut points: "<<this_cane->points[good_vertex_index_[final_bud_seq_index[MIN_BUD_LOCATIONS]]]<<endl;
                //viz.spinOnce();
                //display_graph(g);
                cout<<"#######################################################"<<endl;
            }

            else{
                cout<<"skipping cane.... no root found..... or few bud locations ...."<<endl;
            }

            root_ped_id = -1;
        }
        cout<<"clustering done ..........."<<endl;

        pcl::toROSMsg(*cut_point_cloud_n, output_buds);

        //for (int i = 0; i <3; i++){
            output_buds.header.frame_id = "camera_link";
            output_buds.header.stamp = ros::Time::now();
            pub_cut.publish(output_buds);
            //sleep(1);

            cout<<"publishing cut point done......................."<<endl;
        //}

        //string x__ = to_string(rand()) + "cut point";
        //viz.addPointCloud<pcl::PointXYZRGB>(cut_point_cloud_n, x__);
        //viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,1,0,x__);
        //viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,9.0,x__);
        //viz.addPointCloud<pcl::PointXYZRGB>(cut_point_cloud_n_minus_1, x__ + string ("1"));
        //viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0.64,0,x__ + string ("1"));
        //viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,9.0,x__ +  string ("1"));

        //viz.spinOnce();
        //set_cutpoint_orientation();

        //setup_target();
        //set_cut_point_orientation();

        //viz.spin();
    }

    void return_max_component_graph (vtkmug &g){

        vtkSmartPointer<vtkBoostConnectedComponents> connectedComponents = vtkSmartPointer<vtkBoostConnectedComponents>::New();

        connectedComponents->SetInputData(g);
        connectedComponents->Update();

        vtkGraph* outputGraph = connectedComponents->GetOutput();
        vtkIntArray* components = vtkIntArray::SafeDownCast(outputGraph->GetVertexData()->GetArray("component"));

        //cout<<"No. of connected components: "<<components->GetNumberOfTuples()<<endl;

        std::vector<int> conn_index;

        for(vtkIdType i = 0; i < components->GetNumberOfTuples(); i++) {
          int val = components->GetValue(i);
          //std::cout << val << std::endl;
          conn_index.push_back(val);
        }

        const int i = *std::max_element(conn_index.begin(), conn_index.end());

        //cout<<"no of components: "<<i+1<<endl;

        if (i>0){

            ROS_ERROR_ONCE("Graph has more than 1 component ..... returning the largest sub-graph ......");

            int count = 0;

            std::vector<int>count_vec;

            for(int a = 0 ; a <=i; a++){
                count = 0;
                //vector<int> t;
                for (size_t b =0; b<conn_index.size(); b++){
                    if(a == conn_index[b]){
                        count++;
                    }
                }
                count_vec.push_back(count);
            }

            int max_ind = std::distance(count_vec.begin(), max_element(count_vec.begin(), count_vec.end()));

            vtkSmartPointer<vtkIdTypeArray> ids = vtkSmartPointer<vtkIdTypeArray>::New();

            for (size_t i = 0; i< conn_index.size(); i++ ){
                if (conn_index[i] == max_ind)
                ids->InsertNextValue((vtkIdType)i);
            }

            vtkSmartPointer<vtkSelection> selection = vtkSmartPointer<vtkSelection>::New();
            vtkSmartPointer<vtkSelectionNode> node = vtkSmartPointer<vtkSelectionNode>::New();
            selection->AddNode(node);
            node->SetSelectionList(ids);
            node->SetContentType(vtkSelectionNode::INDICES);
            node->SetFieldType(vtkSelectionNode::VERTEX);
            //node->SetFieldType(vtkSelectionNode::PEDIGREEIDS);
            //node->SetFieldType(vtkSelectionNode::EDGE)

            vtkSmartPointer<vtkExtractSelectedGraph> extractSelectedGraph = vtkSmartPointer<vtkExtractSelectedGraph>::New();
            extractSelectedGraph->SetInputData(0, g);
            extractSelectedGraph->SetInputData(1, selection);
            extractSelectedGraph->Update();

            g->ShallowCopy(extractSelectedGraph->GetOutput());
        }
    }

    void set_cutpoint_orientation(){

        kdtree_cutpoint.setInputCloud(only_cane);

        for(size_t i = 0 ; i < cut_point_cloud_n->points.size(); i++){

            searchPoint = cut_point_cloud_n->points[i];

            pointIdxRadiusSearch.clear();
            pointRadiusSquaredDistance.clear();
            kdtree_cutpoint.radiusSearch (searchPoint, 0.05, pointIdxRadiusSearch, pointRadiusSquaredDistance);

            inliers.clear();
            ransac_sample_cloud->empty();
            final_ransac->empty();

            pcl::copyPointCloud<pcl::PointXYZRGB>(*only_cane, pointIdxRadiusSearch, *ransac_sample_cloud);

            inliers.clear();
            pcl::SampleConsensusModelLine<pcl::PointXYZRGB>::Ptr  model_l (new pcl::SampleConsensusModelLine<pcl::PointXYZRGB> (ransac_sample_cloud));
            pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_l,0.005);
            ransac.computeModel ();
            ransac.getInliers(inliers);

            pcl::copyPointCloud<pcl::PointXYZRGB>(*ransac_sample_cloud, inliers, *final_ransac);

            pcl::copyPointCloud<pcl::PointXYZRGB>(*ransac_sample_cloud, inliers, *ransac_sample_cloud);
            /*
            pcl::SampleConsensusModelCylinder<pcl::PointXYZRGB,pcl::Normal>::Ptr  model_c (new pcl::SampleConsensusModelCylinder<pcl::PointXYZRGB, pcl::Normal> (ransac_sample_cloud));

            // compute normals for cylinder fitting
            pcl::search::Search<pcl::PointXYZRGB>::Ptr tree_cyl = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::Normal>::Ptr normals_cylinder_cmodel(new pcl::PointCloud<pcl::Normal>);
            pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_est_cyl;

            normal_est_cyl.setSearchMethod (tree_cyl);
            normal_est_cyl.setInputCloud (ransac_sample_cloud);
            normal_est_cyl.setKSearch (2);
            normal_est_cyl.compute (*normals_cylinder_cmodel);

            model_c->setInputNormals(normals_cylinder_cmodel);
            pcl::RandomSampleConsensus<PointXYZRGB> sac (model_c, 0.05);

            bool result = sac.computeModel ();

            sac.getInliers (inliers);

            Eigen::VectorXf coeff;
            sac.getModelCoefficients (coeff);

            Eigen::VectorXf coeff_refined;
            model_c->optimizeModelCoefficients (inliers, coeff, coeff_refined);

            pcl::copyPointCloud(*ransac_sample_cloud, inliers, *ransac_sample_cloud);

            string x__ = to_string(rand()) + "cut point";

            cout<<"cylinder size: "<<ransac_sample_cloud->size()<<endl;

            viz.addPointCloud<pcl::PointXYZRGB>(ransac_sample_cloud, x__);
            viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,1,0,x__);
            viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,9.0,x__);
            viz.spin();

            */
            Eigen::Vector4f centroid;
            Eigen::Matrix3f C;

            pcl::compute3DCentroid (*ransac_sample_cloud, centroid);
            pcl::computeCovarianceMatrix (*ransac_sample_cloud, centroid, C);

            cout<<"centroid: "<<centroid<<endl;

            Eigen::Vector3f eigen_values;
            Eigen::Matrix3f eigen_vectors;
            Eigen::Vector3f x_eig(eigen_vectors(0,2), eigen_vectors(1,2), eigen_vectors(2,2));
            Eigen::Vector3f y_eig(eigen_vectors(0,1), eigen_vectors(1,1), eigen_vectors(2,1));
            pcl::eigen33 (C, eigen_vectors, eigen_values);
            Eigen::Vector3f normal(y_eig.cross(x_eig));
           /* PointXYZ Point1 = PointXYZ((centroid(0) + eigen_vectors.col(0)(0)), (centroid(1) + eigen_vectors.col(0)(1)), (centroid(2) + eigen_vectors.col(0)(2)));
            PointXYZ Point2 = PointXYZ((centroid(0) + eigen_vectors(0,1)), (centroid(1) + eigen_vectors(1,1)), (centroid(2) + eigen_vectors(2,1)));
            PointXYZ Point3 = PointXYZ((centroid(0) + eigen_vectors(0,2)), (centroid(1) + eigen_vectors(1,2)), (centroid(2) + eigen_vectors(2,2)));
            PointXYZ Point4;// =
            */
//WORKS FOR STRAIGHT CANES=================================================================================
            Eigen::Vector3f z_prime(eigen_vectors(0,0), eigen_vectors(1,0), eigen_vectors(2,0));
            Eigen::Vector3f y_prime(eigen_vectors(0,1), eigen_vectors(1,1), eigen_vectors(2,1));
            Eigen::Vector3f x_prime(eigen_vectors(0,2), eigen_vectors(1,2), eigen_vectors(2,2));
            Eigen::Matrix3f M;
            M<<x_prime.dot(Eigen::Vector3f(1,0,0)), y_prime.dot(Eigen::Vector3f(1,0,0)), z_prime.dot(Eigen::Vector3f(1,0,0)),
                    x_prime.dot(Eigen::Vector3f(0,1,0)),y_prime.dot(Eigen::Vector3f(0,1,0)),z_prime.dot(Eigen::Vector3f(0,1,0)),
                    x_prime.dot(Eigen::Vector3f(0,0,1)),y_prime.dot(Eigen::Vector3f(0,0,1)),z_prime.dot(Eigen::Vector3f(0,0,1));



            Eigen::Vector3f new_x=M*Eigen::Vector3f(1,0,0)+Eigen::Vector3f(centroid(0),centroid(1),centroid(2));
            Eigen::Vector3f new_y=M*Eigen::Vector3f(0,1,0)+Eigen::Vector3f(centroid(0),centroid(1),centroid(2));
            Eigen::Vector3f new_z=M*Eigen::Vector3f(0,0,1)+Eigen::Vector3f(centroid(0),centroid(1),centroid(2));
            //new_z=(new_x/new_x.norm()).cross(new_y/new_y.norm());
            //cout<<"z :"<<new_z.dot(Eigen::Vector3f(0,0,1))<<" -z: "<<new_z.dot(Eigen::Vector3f(0,0,-1))<<endl;
            //cout<<"y :"<<new_z.dot(Eigen::Vector3f(0,1,0))<<" -y: "<<new_z.dot(Eigen::Vector3f(0,-1,0))<<endl;
            //cout<<"x :"<<new_z.dot(Eigen::Vector3f(1,0,0))<<" -x: "<<new_z.dot(Eigen::Vector3f(-1,0,0))<<endl;
            //cout<<"extra"<<new_z.dot(Eigen::Vector3f(0,1,0).cross(Eigen::Vector3f(1,0,0)))<<endl;
            float sign=1;
            if(new_z.dot(Eigen::Vector3f(0,0,1))<1)
             {  //new_x=M*Eigen::Vector3f(-1,0,0)+Eigen::Vector3f(centroid(0),centroid(1),centroid(2));
                //new_y=M*Eigen::Vector3f(0,-1,0)+Eigen::Vector3f(centroid(0),centroid(1),centroid(2));

                new_z=M*(Eigen::Vector3f(0,1,0).cross(Eigen::Vector3f(1,0,0)))+Eigen::Vector3f(centroid(0),centroid(1),centroid(2));
                 // new_z=M*Eigen::Vector3f(0,0,1)+Eigen::Vector3f(centroid(0),centroid(1),centroid(2));
                 cout<<"Turned"<<endl;
                 sign=-1;

             }
            PointXYZ Point1 = PointXYZ(new_z(0),new_z(1),new_z(2));
            PointXYZ Point2 = PointXYZ(new_y(0),new_y(1),new_y(2));
            PointXYZ Point3 = PointXYZ(new_x(0),new_x(1),new_x(2));

           /* float roll=new_z.dot(Eigen::Vector3f(0,0,1));
            float pitch =new_y.dot(Eigen::Vector3f(0,1,0));
            float yaw =new_x.dot(Eigen::Vector3f(1,0,0));
            cout<<"roll: "<<roll*180/3.1415<<endl;
            cout<<"pitch: "<<pitch*180/3.1415<<endl;
            cout<<"yaw: "<<yaw*180/3.1415<<endl;
*/

           // Eigen::Vector3d x1(0,0,1), y1(0,1,0), z1(1,0,0);

            /*
            Eigen::Quaterniond q =vector2Qt(x1,y1,z1,new_x-Eigen::Vector3d(centroid(0),centroid(1),centroid(2)),

                                     new_y-Eigen::Vector3d(centroid(0),centroid(1),centroid(2)),

                                     new_z-Eigen::Vector3d(centroid(0),centroid(1),centroid(2)));

            */

            //Eigen::Matrix3f R;
            //R<<0, 0, 1, 0, 1, 0, -1 ,0, 0;
            //M = R*M;
            z_prime=M.transpose()*(new_z-Eigen::Vector3f(centroid(0),centroid(1),centroid(2)));
            M<<x_prime.dot(Eigen::Vector3f(1,0,0)), y_prime.dot(Eigen::Vector3f(1,0,0)), z_prime.dot(Eigen::Vector3f(1,0,0)),
                    x_prime.dot(Eigen::Vector3f(0,1,0)),y_prime.dot(Eigen::Vector3f(0,1,0)),z_prime.dot(Eigen::Vector3f(0,1,0)),
                    x_prime.dot(Eigen::Vector3f(0,0,1)),y_prime.dot(Eigen::Vector3f(0,0,1)),z_prime.dot(Eigen::Vector3f(0,0,1));

            Eigen::Vector3f euler = M.eulerAngles(0, 1, 2); // 0,1,2
            //float yaw_ = euler[0]; float pitch_ = euler[1]; float roll_ = euler[2];

            cout<<"Roll: "<< euler[2]*180/3.1415<<endl;
            cout<<"Pitch: "<< euler[1]*180/3.1415<<endl;
            cout<<"Yaw: "<< euler[0]*180/3.1415<<endl;
            cout<<"========================================================"<<endl;

//WORKS FOR STRAIGHT CANES=================================================================================

            //Curved canes solution
       /*     pcl::PointXYZRGB p1;
            pcl::getMinMax3D (*ransac_sample_cloud,p1, p2);
            Eigen::Vector3f cut_d=Eigen::Vector3f(p2.x-p1.x, p2.y-p1.y,p2.z-p1.z);
            Eigen::Matrix3f Rx;
            float theta=(M.transpose()*Eigen::Vector3f(1,0,0)).dot(cut_d/cut_d.norm());
            cout<<"theta: "<<theta<<endl;
            Rx<<1,0,0,
                    0, cos(theta),-sin(theta),
                    0, sin(theta),cos(theta);
            x_prime=Rx*x_prime;
            y_prime=Rx*y_prime;
            z_prime=Rx*z_prime;
            //Eigen::Matrix3f M;
            M<<x_prime.dot(Eigen::Vector3f(1,0,0)), y_prime.dot(Eigen::Vector3f(1,0,0)), z_prime.dot(Eigen::Vector3f(1,0,0)),
                    x_prime.dot(Eigen::Vector3f(0,1,0)),y_prime.dot(Eigen::Vector3f(0,1,0)),z_prime.dot(Eigen::Vector3f(0,1,0)),
                    x_prime.dot(Eigen::Vector3f(0,0,1)),y_prime.dot(Eigen::Vector3f(0,0,1)),z_prime.dot(Eigen::Vector3f(0,0,1));

            Eigen::Vector3f new_x=M.transpose()*Eigen::Vector3f(1,0,0)+Eigen::Vector3f(centroid(0),centroid(1),centroid(2));
            Eigen::Vector3f new_y=M.transpose()*Eigen::Vector3f(0,1,0)+Eigen::Vector3f(centroid(0),centroid(1),centroid(2));
            Eigen::Vector3f new_z=M.transpose()*Eigen::Vector3f(0,0,-1)+Eigen::Vector3f(centroid(0),centroid(1),centroid(2));
            PointXYZ Point1 = PointXYZ(new_z(0),new_z(1),new_z(2));
            PointXYZ Point2 = PointXYZ(new_y(0),new_y(1),new_y(2));
            PointXYZ Point3 = PointXYZ(new_x(0),new_x(1),new_x(2));

*/


            ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("pose", 1000);

            cout<<"here ....1."<<endl;
            geometry_msgs::PoseStamped pose;

            pose.pose.position.x = centroid(0);
            pose.pose.position.y = centroid(1);
            pose.pose.position.z = centroid(2);

            cout<<"here ....2."<<endl;

            tf::Quaternion Q;

            Q = tf::createQuaternionFromRPY(euler[0],euler[1],euler[2]); //cut 0
            pose.pose.orientation.x = Q.x();
            pose.pose.orientation.y = Q.y();
            pose.pose.orientation.z = Q.z();
            pose.pose.orientation.w = Q.w();

            cout<<"pose converted ......."<<endl;

            for (int u = 0 ; u < 2; u++){

                pose.header.frame_id = "map";
                pose.header.stamp = ros::Time::now();
                pub_pose.publish(pose);
                sleep(1);
                cout<<"publishing pose ......"<<endl;

            }

           /* if (z_bud.dot(Eigen::Vector3f(0,0,1))<0 || y_bud.dot(Eigen::Vector3f(0,0,1))<0 ){
                if (abs(z_bud.dot(Eigen::Vector3f(0,0,1)))>abs(y_bud.dot(Eigen::Vector3f(0,0,1))))
                {
                    Point1=  PointXYZ(centroid(0)-eigen_vectors(0,0), centroid(1)-eigen_vectors(1,0),centroid(2)+eigen_vectors(2,0));
                    cout<<"Turned y"<<endl;
                }
                else{
                Point2=  PointXYZ(centroid(0)-eigen_vectors(0,0), centroid(1)+eigen_vectors(1,0),centroid(2)-eigen_vectors(2,0));
                cout<<"Turned z"<<endl;
                }
                //Point4=PointXYZ((centroid(0) - normal(0)), (centroid(1) + normal(1)), (centroid(2) - normal(2)));;
            }

            else
               { Point1 = PointXYZ(centroid(0)+eigen_vectors(0,0), centroid(1)+eigen_vectors(1,0),centroid(2)+eigen_vectors(2,0));

               // Point4 = PointXYZ((centroid(0) + normal(0)), (centroid(1) + normal(1)), (centroid(2) + normal(2)));
            }*/
            PointXYZ centroidXYZ;
            centroidXYZ.getVector4fMap() = centroid;

            //viz.addCoordinateSystem(1.0,centroid(0),centroid(1),centroid(2));

            //viz.addArrow(Point1,centroidXYZ,1,0,0, false,to_string(i) + "arrow1");
            //viz.addArrow(Point2,centroidXYZ,0,1,0, false,to_string(i) + "arrow2");
            //viz.addArrow(Point3,centroidXYZ,0,0,1, false,to_string(i) + "arrow3");
            //viz.addArrow(Point4,centroidXYZ,1,0,1, false,to_string(i) + "arrow4");

            //viz.addPointCloud<pcl::PointXYZRGB>(ransac_sample_cloud, std::to_string(i) + "ransac");
            //viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,1,0,std::to_string(i) + "ransac");

            //viz.spinOnce();

            //viz.addArrow(Point1,centroidXYZ,0,0,0, false,to_string(i) + "arrow1_");
            //viz.addArrow(Point2,centroidXYZ,0,0,0, false,to_string(i) + "arrow2_");
            //viz.addArrow(Point3,centroidXYZ,0,0,0, false,to_string(i) + "arrow3_");
            //viz.addArrow(Point4,centroidXYZ,1,0,1, false,to_string(i) + "arrow4_");
            //viz.spin();
        }
        //viz.spin();
    }

    int point_2_vertex(pcl::PointXYZRGB p_, pcl::PointCloud<pcl::PointXYZRGB>::Ptr P){

        int counter = 0;
        // this function takes a pcl::pointXYZRGB and finds the associated vertex ID in the precimputed graph
       std::vector<int> pointIdxVec;
       if (oCtree.voxelSearch (p_, pointIdxVec)){

           //std::cout << "Neighbors within voxel search at (" << searchPoint.x<< " " << searchPoint.y<< " " << searchPoint.z << ")"<< std::endl;
           //cout<<"octree index: "<<pointIdxVec

           for (size_t i = 0; i < pointIdxVec.size (); ++i){
               if (P->points[pointIdxVec[i]].r==255){

                   //cout<<"counter: "<<counter <<" i "<<i<<"  vector: "<<pointIdxVec[i]<<endl;
                   std::vector<int>::iterator it;
                   it = std::find(good_vertex_index_.begin(),good_vertex_index_.end(),pointIdxVec[i]);


                   //return pointIdxVec[i]; // in case of two red/ yellow points, use only one and discard the other
                                          // will only return for red/ yellow points
                   int index = std::distance(good_vertex_index_.begin(), it);

                   return good_vertex_index_[index];
               }
                counter++;
               //std::cout << "point: " << P->points[pointIdxVec[i]]<<" Index: "<<pointIdxVec[i]<< std::endl;
           }
       }

    }

    vtkmug generate_graph(pcl::PointCloud<pcl::PointXYZRGB> cloud, float res_){

        clock_t begin = clock();
        // make sure that the cloud is the voxel cloud not the sensor input point cloud
        vtkmug g = vtkmug::New();
        Eigen::MatrixXf coor;
        coor.resize(int(cloud.points.size()),3);

        vtkSmartPointer<vtkVariantArray> vertex_pedigree_id = vtkSmartPointer<vtkVariantArray>::New();
        vertex_pedigree_id->SetName("vertex_pedigree_ids");
        g->GetVertexData()->SetPedigreeIds(vertex_pedigree_id);

        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        double d[3];

        // Create the color array
        vtkSmartPointer<vtkIntArray> vertexColors = vtkSmartPointer<vtkIntArray>::New();
        vertexColors->SetNumberOfComponents(1);
        vertexColors->SetName("Color");


        vtkSmartPointer<vtkLookupTable> lookupTable = vtkSmartPointer<vtkLookupTable>::New();

        const int n = cloud.points.size();
        lookupTable->SetNumberOfTableValues(n);
        lookupTable->SetTableValue(0, 1.0, 0.0, 0.0); // red
        lookupTable->Build();

        std::vector< std::tuple<int,string,int>> extreme_node;

        //#pragma omp parallel num_threads(3)
        for (size_t i = 0 ; i <cloud.points.size(); i++){// adding Pedigree ID for vertex
            //char ch[10];
            //snprintf(ch,sizeof(ch),"%04d",(int)i+1);
            //std::string c = "N_"  + string(ch);
            //vertex_pedigree_id->InsertNextValue(c.c_str());
            //g->AddVertex(c.c_str()); // adding pedigree id to the vertex
            vertex_pedigree_id->InsertNextValue(pedg_id[i].c_str());
            g->AddVertex(pedg_id[i].c_str());
            vertexColors->InsertNextValue(i);
            //g->GetVertexData()->SetScalars(colors);
            coor(i,0)  = cloud.points[i].x;
            coor(i,1)  = cloud.points[i].y;
            coor(i,2)  = cloud.points[i].z;
            points->InsertNextPoint(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);

            if (cloud.points[i].r >= 250)
                extreme_node.push_back(std::make_tuple(1,pedg_id[i].c_str() ,1));
            points->GetPoint(i,d);
            //cout<<"cloud["<<i<<"]:"<<cloud.points[i]<<" ped_id: "<<pedg_id[i].c_str()<<" graph_point: "<<d[0]<<", "<<d[1]<<", "<<d[2]<<endl;

        }

        //cout <<"extreme nodes ...........server "<<endl;

        //for (size_t i = 0 ; i <extreme_node.size(); i++)
            //cout<<std::get<0>(extreme_node[i])<<", "<<std::get<1>(extreme_node[i])<<", "<<std::get<2>(extreme_node[i])<<endl;

        g->GetVertexData()->AddArray(vertexColors);
        std::vector< std::pair<int, int> > edge_list = find_edges(cloud, res_); //graph_res

        #pragma omp parallel num_threads(3)
        for (size_t i = 0; i <edge_list.size(); i++){ // add edge using the above pedigree IDs
            g->AddEdge(g->GetVertexData()->GetPedigreeIds()->GetVariantValue(edge_list[i].first), g->GetVertexData()->GetPedigreeIds()->GetVariantValue(edge_list[i].second));
        }
        g->SetPoints(points);

        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        //cout<<" generate graph time elapsed: "<<elapsed_secs<<endl;


/*
        Eigen::MatrixXf _sq = squareform(coor);
        Eigen::MatrixXf::Index maxRow, maxCol;//, minRow, minCol;
        _sq.maxCoeff(&maxRow, &maxCol);

        cout << "_sq_max at: " <<maxRow << ", " << maxCol << endl;

        extreme_val_1 = maxRow;
        extreme_val_2 = maxCol;
*/
        vtkmug g_ = vtkmug::New();
        g_ = simplify_edge(g);

        //vtkmug gx = simplify_xdof(g_,coor, extreme_node);
        return g_;
    }

    vtkmug simplify_xdof (vtkmug g, Eigen::MatrixXf coor, std::vector< std::tuple<int,string,int>> extreme_node){


        //cout <<"extreme nodes ...........client "<<endl;

        //for (size_t i = 0 ; i <extreme_node.size(); i++)
        //    cout<<std::get<0>(extreme_node[i])<<", "<<std::get<1>(extreme_node[i])<<", "<<std::get<2>(extreme_node[i])<<endl;

        vtkSmartPointer<vtkVertexListIterator> v_i = vtkSmartPointer<vtkVertexListIterator>::New();
        std::vector< std::tuple<int,string,int>> delete_list_, delete_list;
        std::vector<string> g_vertex_vec;

        g->GetVertices(v_i);

        while (v_i->HasNext()){

            vtkIdType v = v_i->Next();
            //delete_list_.push_back(std::make_tuple(g->GetDegree(v),g->GetVertexData()->GetPedigreeIds()->GetVariantValue(v).ToString(),int(v)));
            g_vertex_vec.push_back(string(g->GetVertexData()->GetPedigreeIds()->GetVariantValue(v).ToString()));

            if (g->GetDegree(v) == 1){
                std::tuple<int,string,int> t;
                t = std::make_tuple(g->GetDegree(v),g->GetVertexData()->GetPedigreeIds()->GetVariantValue(v).ToString(),int(v));
                extreme_node.push_back(t);
            }
        }

        vtkmug g_ = vtkmug::New();
        g_->DeepCopy(g);

        Eigen::MatrixXf _sq = squareform(coor);
        Eigen::MatrixXf::Index maxRow, maxCol;//, minRow, minCol;
        _sq.maxCoeff(&maxRow, &maxCol);

        //cout << "_sq_max: " << max_ <<  ", at: " <<maxRow << ", " << maxCol << endl;
        //extreme_node.push_back(std::make_tuple(g->GetDegree(maxRow),g->GetVertexData()->GetPedigreeIds()->GetVariantValue(maxRow).ToString(),int(maxRow)));
        //extreme_node.push_back(std::make_tuple(g->GetDegree(maxCol),g->GetVertexData()->GetPedigreeIds()->GetVariantValue(maxCol).ToString(),int(maxCol)));

        extreme_node.push_back(std::make_tuple(g->GetDegree(maxRow),g->GetVertexData()->GetPedigreeIds()->GetVariantValue(maxRow).ToString(),int(maxRow)));
        extreme_node.push_back(std::make_tuple(g->GetDegree(maxCol),g->GetVertexData()->GetPedigreeIds()->GetVariantValue(maxCol).ToString(),int(maxCol)));

        std::sort(extreme_node.begin(), extreme_node.end());
        extreme_node.erase( unique( extreme_node.begin(), extreme_node.end() ), extreme_node.end() );// remove duplicate values


        //cout<<"extreme dof == 1"<<endl;

        //for (size_t i = 0 ; i <extreme_node.size(); i++)
        //    cout<<std::get<0>(extreme_node[i])<<", "<<std::get<1>(extreme_node[i])<<", "<<std::get<2>(extreme_node[i])<<endl;

        int done = 0;
        while(done == 0){

            int node_count_before = int(g_->GetNumberOfVertices());

            vtkSmartPointer<vtkVertexListIterator> v_it = vtkSmartPointer<vtkVertexListIterator>::New();
            vtkSmartPointer<vtkIdTypeArray> vertices_1dof =vtkSmartPointer<vtkIdTypeArray> ::New();
            vtkSmartPointer<vtkVariantArray> va = vtkSmartPointer<vtkVariantArray>::New();

            g_->GetVertices(v_it);

            while(v_it->HasNext()){
                vtkIdType ver = v_it->Next();
                vtkIdType v_dof = g_->GetDegree(ver);
                if (v_dof == 1){
                    vertices_1dof->InsertNextValue(ver);
                    va->InsertNextValue(g_->GetVertexData()->GetPedigreeIds()->GetVariantValue(ver));
                }
            }

            if (vertices_1dof->GetNumberOfTuples()>0){

                //g_->RemoveVertices(vertices_1dof);
                g_ = RemoveVerticesPids(g_,va );

            }

            int node_count_after = int(g_->GetNumberOfVertices());

            if (node_count_before == node_count_after)
                done = 1;
        }

        vtkmug gg = vtkmug::New();
        gg = sort_graph(g_);

        gg->GetVertices(v_i);

        Eigen::MatrixXd DOF_G1, DOF_G2, DoF_Diff;

        DOF_G1.resize(int(gg->GetNumberOfVertices()),1);
        DOF_G2.resize(int(gg->GetNumberOfVertices()),1);
        DoF_Diff.resize(int(gg->GetNumberOfVertices()),1);

        // g1 is the processed graph here gg
        // g2 is the predecessor graph here g or g_

        std::vector< std::tuple<int,string,int>> gate_nodes;
        std::vector<std::tuple<int,string,int>> g1_nodes;

        int counter = 0;

        while(v_i->HasNext()){
            vtkIdType v = v_i->Next();
            std::vector<string>::iterator it = std::find(g_vertex_vec.begin(),g_vertex_vec.end(),gg->GetVertexData()->GetPedigreeIds()->GetVariantValue(v).ToString());
            if (it != g_vertex_vec.cend()) {
                DOF_G1(counter,0) = gg->GetDegree(v);
                DOF_G2(counter,0) = g->GetDegree(vtkIdType(std::distance(g_vertex_vec.begin(), it)));
                counter++;
            }

            std::tuple<int,string,int> p;
            p = std::make_tuple(gg->GetDegree(v),gg->GetVertexData()->GetPedigreeIds()->GetVariantValue(v).ToString(),int(v));
            g1_nodes.push_back(p);
        }

    //    cout<<"g1_nodes: "<<endl;
    //    for(size_t i = 0; i<g1_nodes.size(); i++)
    //        cout<<std::get<0>(g1_nodes[i])<<", "<<std::get<1>(g1_nodes[i])<<", "<<std::get<2>(g1_nodes[i])<<endl;
    //    cout<<"------------------------------------"<<endl;

        DoF_Diff = DOF_G1 - DOF_G2;
        DoF_Diff = DoF_Diff.array().abs(); // equivalent to asb() c++

        for(Eigen::Index i = 0; i<DoF_Diff.size(); ++i){
            if(DoF_Diff(i)){
                std::tuple<int,string,int> p;
                p = std::make_tuple(gg->GetDegree(i),gg->GetVertexData()->GetPedigreeIds()->GetVariantValue(i).ToString(),int(i));
                gate_nodes.push_back(p);
            }
        }

//        cout<<"gate_nodesbefore : "<<endl;
//        for(size_t it = 0; it<gate_nodes.size(); it++)
//            cout<<std::get<0>(gate_nodes[it])<<", "<<std::get<1>(gate_nodes[it])<<", "<<std::get<2>(gate_nodes[it])<<endl;
//        cout<<"------------------------------------"<<endl;

        gate_nodes.insert(gate_nodes.end(), extreme_node.begin(),extreme_node.end());

//        cout<<"gate_nodes after: "<<endl;
//        for(size_t it = 0; it<gate_nodes.size(); it++)
//            cout<<std::get<0>(gate_nodes[it])<<", "<<std::get<1>(gate_nodes[it])<<", "<<std::get<2>(gate_nodes[it])<<endl;
//        cout<<"------------------------------------"<<endl;

        //std::set_difference(delete_list_.begin(), delete_list_.end(),gate_nodes.begin(), gate_nodes.end(),std::back_inserter(delete_list));


        vector<std::tuple<int,string,int> > nodes_available;
        std::sort(g1_nodes.begin(),g1_nodes.end());
        std::sort(gate_nodes.begin(),gate_nodes.end());
        std::set_difference(g1_nodes.begin(),g1_nodes.end(),gate_nodes.begin(),gate_nodes.end(),std::back_inserter(nodes_available));

        delete_list_ = nodes_available;

//        cout<<"delete_list_(size): "<<delete_list_.size()<<endl;
    //    cout<<"nodes_available: "<<endl;
    //    for (size_t i = 0; i< nodes_available.size(); i++)
    //        cout<<std::get<0>(nodes_available[i])<<", "<<std::get<1>(nodes_available[i])<<", "<<std::get<2>(nodes_available[i])<<endl;
    //    cout<<"------------------------------------"<<endl;
        std::vector< std::tuple<int,string,int>> do_not_delete_list = gate_nodes;
    //    for (size_t ii = 0 ; ii <do_not_delete_list.size(); ii++)
    //        cout<<"do_not_delete_list: "<<std::get<0>(do_not_delete_list[ii])<<", "<<std::get<1>(do_not_delete_list[ii])<<", "<<std::get<2>(do_not_delete_list[ii])<<endl;

    //    cout<<"------------------------------------"<<endl;
        done = 0;

    //    gg->GetVertices(v_i);

    //    while(v_i->HasNext()){
    //        vtkIdType v = v_i->Next();
    //        cout<<"gg_verted data("<<v<<") -> "<<gg->GetVertexData()->GetPedigreeIds()->GetVariantValue(v)<<endl;
    //    }

        while(done == 0){

            if(nodes_available.size()){
                auto min = std::min_element(nodes_available.begin(),nodes_available.end());

    //            cout<<"Min: "<<endl;
    //            cout<<std::get<0>((*min))<<endl;
    //            cout<<std::get<1>((*min))<<endl;
    //            cout<<std::get<2>((*min))<<endl;

                std::tuple<int,string,int>del{std::get<0>((*min)), std::get<1>((*min)), std::get<2>((*min))};
                std::vector<std::tuple<int,string,int>>del_;
                del_.push_back(del);

                //cout<<"del_: "<<std::get<2>(del_[0])<<", "<<std::get<1>(del_[0])<<endl;

                std::vector<std::tuple<int,string,int>>ismem;
                std::set_intersection(del_.begin(),del_.end(),do_not_delete_list.begin(),do_not_delete_list.end(),std::back_inserter(ismem));

                if(!ismem.size()){ //ismember

                    vtkmug g__ = vtkmug::New();
                    g__->DeepCopy(gg);
                    vtkSmartPointer<vtkVariantArray> vac = vtkSmartPointer<vtkVariantArray>::New();
                    vac->InsertNextValue(std::get<1>(del_[0]).c_str());
                    g__ = RemoveVerticesPids(g__, vac);

                    if (still_connected(g__)){
                        //cout<<"     Removing node:["<<std::get<2>(del_[0])<<", "<<std::get<1>(del_[0]).c_str()<<"]"<<endl;
                        vtkSmartPointer<vtkVariantArray> va = vtkSmartPointer<vtkVariantArray>::New();
                        va->InsertNextValue(std::get<1>(del_[0]).c_str());
                        gg = RemoveVerticesPids(gg, va); //????????????????

                        std::tuple<int,string,int> a {std::get<0>(del_[0]),std::get<1>(del_[0]).c_str(),std::get<2>(del_[0])};
                        nodes_available.erase(std::remove(nodes_available.begin(),nodes_available.end(),a),nodes_available.end());
                    }

                    else{
                        do_not_delete_list.push_back(del_[0]);
                        std::tuple<int,string,int> a {std::get<0>(del_[0]),std::get<1>(del_[0]).c_str(),std::get<2>(del_[0])};
                        nodes_available.erase(std::remove(nodes_available.begin(),nodes_available.end(),a),nodes_available.end());
                    }
                }
            }

            else{
                done = 1;
            }
        }

//        for (size_t ii = 0 ; ii <do_not_delete_list.size(); ii++)
//            cout<<"do_not_delete_list("<<ii<<"): "<<std::get<0>(do_not_delete_list[ii])<<", "<<std::get<1>(do_not_delete_list[ii])<<", "<<std::get<2>(do_not_delete_list[ii])<<endl;
//        cout<<"------------------------------------"<<endl;
        std::sort(delete_list_.begin(), delete_list_.end());
        std::sort(do_not_delete_list.begin(), do_not_delete_list.end());
        std::set_difference(delete_list_.begin(), delete_list_.end(),do_not_delete_list.begin(), do_not_delete_list.end(),std::back_inserter(delete_list));
//        cout<<"delete_list(size): "<<delete_list.size()<<endl;

        vtkSmartPointer<vtkVariantArray> del  = vtkSmartPointer<vtkVariantArray>::New();
        for (size_t ii = 0 ; ii <delete_list.size(); ii++){
            del->InsertNextValue(std::get<1>(delete_list[ii]).c_str());
            //cout<<"del("<<ii<<"): "<<std::get<1>(delete_list[ii]).c_str()<<endl;
        }

        //vtkmug G = vtkmug::New();
        //G->DeepCopy(g);
        g = RemoveVerticesPids(g,del);

        //display_graph(g);
        return g;
    }

    bool is_connected(vtkmug graph, int start_vertex_idx, int end_vertex_idx, int & length){

        // use Dijkstra's path search between two pints here

        cout<<"start: "<<start_vertex_idx<<" end: "<<end_vertex_idx<<endl;

        vtkSmartPointer<vtkGraphToPolyData> graphToPolyData = vtkSmartPointer<vtkGraphToPolyData>::New();
        graphToPolyData->SetInputData(graph);
        graphToPolyData->Update();

        //vtkVariant p_id = start_vertex_idx;
        cout<<"graph->FindVertex(start_vertex_idx):"<<graph->FindVertex(start_vertex_idx)<<endl;
        cout<<"graph->FindVertex(end_vertex_idx):"<<graph->FindVertex(end_vertex_idx)<<endl;
        cout<<"graph->start_vertex_point: "<<*graph->GetPoint(graph->FindVertex(start_vertex_idx))<<endl;// GetVertexData()->GetPedigreeIds()->GetVariantValue()<<endl;


        //cout<<"graph->GetTargetVertex(end_vertex_idx): "<<graph->GetTargetVertex(end_vertex_idx)<<endl;
        vtkSmartPointer<vtkDijkstraGraphGeodesicPath> dijkstra =  vtkSmartPointer<vtkDijkstraGraphGeodesicPath>::New();
        dijkstra->SetInputConnection(graphToPolyData->GetOutputPort());
        dijkstra->SetStartVertex(graph->FindVertex(start_vertex_idx));
        dijkstra->SetEndVertex(graph->FindVertex(end_vertex_idx));
        dijkstra->Update();

        const int size = dijkstra->GetIdList()->GetNumberOfIds();
        length = size;


        cout<<"no. of IDS: "<<size<<endl;

        /*
        for (int i = 0; i<size; i++ ){

            double point[3];
            dijkstra->GetOutput()->GetPoint(i,point);
            cout<<dijkstra->GetIdList()->GetId(i)<<"\tval: "<<point[0]<<", \t"<<point[1]<<", \t"<<point[2]<<endl;
        }
        */

        // Create a mapper and actor
        vtkSmartPointer<vtkPolyDataMapper> pathMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        pathMapper->SetInputConnection(dijkstra->GetOutputPort());
        //cout<<"no of points: "<<dijkstra->GetOutput()->GetNumberOfPoints()<<endl;

        vtkSmartPointer<vtkActor> pathActor = vtkSmartPointer<vtkActor>::New();
        pathActor->SetMapper(pathMapper);
        pathActor->GetProperty()->SetColor((rand()%255),(rand()%255),(rand()%255)); // Red
        pathActor->GetProperty()->SetLineWidth(4);

        // Create a mapper and actor
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(graphToPolyData->GetOutputPort());

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);

        //viz.getRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actor);
        //viz.getRenderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(pathActor);
        //viz.getRenderWindow()->Render();
        //viz.spin();



        if(size == 0)
            return false;

        return true;
    }

    inline bool is_all_blue(pcl::PointXYZRGB p, pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree, float radius ){

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        //float radius = 0.05;

        kdtree.radiusSearch (p, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

        int count_blue = 0;

        #pragma omp parallel
        for (size_t j = 0; j < pointIdxRadiusSearch.size (); j++){
            if (no_nan_cloud->points[pointIdxRadiusSearch[j]].r == 0 &&
                no_nan_cloud->points[pointIdxRadiusSearch[j]].g == 0 &&
                no_nan_cloud->points[pointIdxRadiusSearch[j]].b == 255){
                count_blue++;
            }
        }

        if (count_blue == (int)pointIdxRadiusSearch.size()){
            return true;
        }

        else{
            return false;
        }
    }

    inline bool is_all_blue_few_red(pcl::PointXYZRGB p, pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree, float radius ){

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        //float radius = 0.05;

        kdtree.radiusSearch (p, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

        int count_blue = 0;
        int count_red = 0;

        if(pointIdxRadiusSearch.size () <= 1 )
            return false;

        #pragma omp parallel
        for (size_t j = 0; j < pointIdxRadiusSearch.size (); j++){
            //cout<<"RGB: "<<no_nan_cloud->points[pointIdxRadiusSearch[j]]<<endl;

            if (no_nan_cloud->points[pointIdxRadiusSearch[j]].r == 0 &&
                no_nan_cloud->points[pointIdxRadiusSearch[j]].g == 0 &&
                no_nan_cloud->points[pointIdxRadiusSearch[j]].b == 255){
                count_blue++;
            }
            if (no_nan_cloud->points[pointIdxRadiusSearch[j]].r == 255 &&
                no_nan_cloud->points[pointIdxRadiusSearch[j]].g == 0 &&
                no_nan_cloud->points[pointIdxRadiusSearch[j]].b == 0){
                count_red++;
            }
        }

        //cout<<" Neighbour size: "<<pointIdxRadiusSearch.size ()<<" red count: "<<count_red<<" blue count: "<<count_blue;

        if ( count_blue == (int)pointIdxRadiusSearch.size() || abs( (int)pointIdxRadiusSearch.size() - count_blue) <=2 ){ //count_blue == (int)pointIdxRadiusSearch.size()
            //cout<<" true \n"<<"----------------------------"<<endl;
            return true;
        }

        else{
            //cout<<" false \n"<<"----------------------------"<<endl;
            return false;
        }


    }

    void eudlidean_cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &P){

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_input (new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree_input->setInputCloud (P);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance (0.02);// 0.05 //0.2 = 2cm
        ec.setMinClusterSize (1);
        ec.setMaxClusterSize (1000);
        ec.setSearchMethod (tree_input);
        ec.setInputCloud (P);
        ec.extract (cluster_indices);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr mean_cl (new pcl::PointCloud<pcl::PointXYZRGB>);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){

            float average_x, average_y, average_z;
            float sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;

            int counter = 0;

            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){


                pcl::PointXYZRGB p = P->points[*pit];

                sum_x = sum_x + p.x;
                sum_y = sum_y + p.y;
                sum_z = sum_z + p.z;

                counter++;
            }

            average_x = (float) sum_x /  counter;
            average_y = (float) sum_y /  counter;
            average_z = (float) sum_z /  counter;

            pcl::PointXYZRGB mean_p;
            mean_p.x = average_x;
            mean_p.y = average_y;
            mean_p.z = average_z;
            mean_p.r = 255; mean_p.g = 255; mean_p.b = 0;
            mean_cl->points.push_back(mean_p);
        }
        P->clear();
        pcl::copyPointCloud(*mean_cl,*P);
    }

    void fit3Dlines(float thresh=0.01){

        float x_val=1.0;
        vector<int> v_inliers;
        vector<int> h_inliers;
        Eigen::VectorXf wire,cordon;

        Eigen::Vector3f ax,ax2;
        ax<<1.0,0.0,0.0;
        pcl::SampleConsensusModelParallelLine<pcl::PointXYZRGB>::Ptr model_l (new pcl::SampleConsensusModelParallelLine<PointXYZRGB> (no_nan_cloud->makeShared ()));
        model_l->setAxis(ax);
        model_l->setEpsAngle(pcl::deg2rad (10.0));
        pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_l);
        ransac.setDistanceThreshold (thresh);
        ransac.computeModel();
        ransac.getInliers(h_inliers);
        ransac.getModelCoefficients(wire);


        ax2<<0.0,1.0,0.0;
        SampleConsensusModelParallelLine<pcl::PointXYZRGB>::Ptr model_l2 (new SampleConsensusModelParallelLine<PointXYZRGB> (no_nan_cloud->makeShared ()));
        model_l2->setAxis(ax2);
        model_l2->setEpsAngle(pcl::deg2rad (10.0));
        pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac2 (model_l2);
        ransac2.setDistanceThreshold (thresh);
        ransac2.computeModel();
        ransac2.getInliers(v_inliers);
        ransac2.getModelCoefficients(cordon);

        //fitted_lines::fitted_lines f(h_inliers,v_inliers,wire,cordon);
        //return f;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr WIRES( new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr CORDON( new pcl::PointCloud<pcl::PointXYZRGB>);

        float x,y,z;
        pcl::copyPointCloud<pcl::PointXYZRGB>(*no_nan_cloud, h_inliers, *WIRES);
        //int w_c_size=WIRES->points.size();

        pcl::copyPointCloud<pcl::PointXYZRGB>(*no_nan_cloud, v_inliers, *CORDON);

        //viz.addPointCloud(CORDON,"cordon");
        //viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,1,0,"cordon");

        //viz.spin();

        float t=2.5;
        int done = 0;

        //cout<<"wire: "<<wire<<endl;

        //for(int i=0;i<150;i++){ // 350
            //t -=0.025; // 0.01

        float length = 0.0;
        float dL = 0.01;
        while(done == 0){

            pcl::PointXYZRGB r;
            r.x = wire[0]+t*wire[3];
            r.y = wire[1]+t*wire[4];
            r.z = wire[2]+t*wire[5];

            r.r = 0; r.g=255; r.b=0;

            //cout<<"wire: "<<r<<endl;

            //wires_fill->points.push_back(pcl::PointXYZRGB(wire[0]+t*wire[3],wire[1]+t*wire[4],wire[2]+t*wire[5]));
            wires_fill->points.push_back(r);
            t = t - dL;

            length = length + dL;
            if (length > 3)
                done = 1;
            //cout<<"t: "<<t<<endl;
        }

        /*
        for (int i=0;i<150;i++){ // 350
            x=wires_fill->points[i].x+x_val;
            y=wires_fill->points[i].y;
            z=wires_fill->points[i].z;

            pcl::PointXYZRGB r;
            r.x = x;
            r.y = y;
            r.z = z;

            r.r = 0; r.g=255; r.b=0;

           // wires_fill->points.push_back(pcl::PointXYZRGB(x,y,z));
             wires_fill->points.push_back(r);
        }
        */

        //cout<<"wire_fill size: "<<wires_fill->points.size()<<endl;

        //display_pcl(wires_fill);
    }

    vtkmug simplify_edge(vtkmug g){

         /*
         * Removes Redundant edges at the begenning when connectivity is generated from the point cloud
         * from Octree Voxel Centroid
         */

        vtkmug g_ = vtkmug::New();
        vtkSmartPointer<vtkEdgeListIterator> edge_it =vtkSmartPointer<vtkEdgeListIterator>::New();
        g_->DeepCopy(g);  // this copies the edge, vertex along with pedigree info
        g_->GetEdges(edge_it);

        std::vector< std::pair<int,int> >edge_list_vector;
        vtkSmartPointer<vtkIdTypeArray> all_edges = vtkSmartPointer<vtkIdTypeArray>::New();

        while (edge_it->HasNext()){
            std::pair<int,int> p;
            vtkEdgeType edge = edge_it->Next();
            all_edges->InsertNextValue(edge.Id);
            p.first = edge.Source;
            p.second = edge.Target;
            edge_list_vector.push_back(p);
        }

        std::sort(edge_list_vector.begin(),edge_list_vector.end()); // sorting do that duplicates can be removed
        edge_list_vector.erase(std::unique(edge_list_vector.begin(),edge_list_vector.end()),edge_list_vector.end()); // remove duplicate entries
                                                                                                                     // vtk by deafult lists duplicate entries
                                                                                                                     // such that unique() can be used to remove it
        g_->RemoveEdges(all_edges); // removing edges to add new no-duplicate edge list

        for (size_t i = 0; i<edge_list_vector.size(); i++){
            g_->AddEdge(edge_list_vector[i].first, edge_list_vector[i].second); // copy edge list
        }

        return g_;
    }

    std::vector< std::pair<int, int>> find_edges (pcl::PointCloud<pcl::PointXYZRGB> coor, float res){

        clock_t begin = clock();
        std::vector< std::pair<int, int> >result;
        Eigen::MatrixXf f(6,3); // The kernell for 6 face connectivity
        Eigen::MatrixXf c(1,3);

        f<<res, 0, 0, -res,0, 0, 0, res, 0, 0,-res, 0, 0, 0, res, 0, 0, -res;

        std::vector<std::tuple<float, float, float>> coor_t;

        //#pragma omp parallel for
        //#pragma omp parallel num_threads(3)
        for(size_t i = 0; i < coor.points.size(); i++){ // converting pcl to tuple
            coor_t.push_back(std::make_tuple(coor.points[i].x, coor.points[i].y ,coor.points[i].z));
        }

        pcl::PointXYZRGB p;
        Eigen::MatrixXf temp(1,3);
        std::tuple<float,float,float> t;
        std::pair<bool, int> var;
        std::pair<int,int> edges;

        for(size_t i = 0; i < coor_t.size(); i++){

            p = coor.points[i];
            temp<<p.x,p.y,p.z;

            for(int j = 0; j<6; j++){

                c<< 0.0, 0.0, 0.0;
                c<<f.row(j);
                c = c + temp;
                //std::tuple<float,float,float> t;
                t  = std::make_tuple(c(0,0), c(0,1), c(0,2));
                var = ismembertol(coor_t,t,0.0001);

                if(var.first){
                    edges.first = i;
                    edges.second =var.second;
                    result.push_back(edges);
                }
            }
        }

        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        //cout<<" find edges time elapsed: "<<elapsed_secs<<endl;

        return result; // later make sure to have code that handles empty vector for no edge list
    }

    vtkmug RemoveVerticesPids(vtkmug g, vtkVariantArray *PId){

        vtkmug g_ = vtkmug::New();
        std::vector<string> pid_g;

        vtkSmartPointer<vtkIdTypeArray> PId_ = vtkSmartPointer<vtkIdTypeArray>::New();
        vtkSmartPointer<vtkVertexListIterator> v_i = vtkSmartPointer<vtkVertexListIterator>::New();

        g_->DeepCopy(g);
        g->GetVertices(v_i);

        while(v_i->HasNext()){
            vtkIdType v = v_i->Next();
            std::string s = g_->GetVertexData()->GetPedigreeIds()->GetVariantValue(v).ToString();
            //cout<<"Removing.... "<<s<<endl;
            pid_g.push_back(s);
        }

        for (int i = 0 ; i <PId->GetNumberOfTuples(); i++){
            std::string s = PId->GetValue(i).ToString();

            for(size_t j = 0 ; j < pid_g.size(); j++){

                if(s == pid_g[j]){
                   //cout<<"Removing.... "<<s<<endl;
                   PId_->InsertNextValue(int(j));
                }
            }
        }

        g_->RemoveVertices(PId_);

        //vtkmug g__ = vtkmug::New();
        //g__->DeepCopy(g_);

        g_  = sort_graph (g_);
        //g__->GetVertices(v_i);
        //cout<<"g_ vertex count: "<<g__->GetNumberOfVertices()<<endl;
//        while(v_i->HasNext()){
//            vtkIdType v = v_i->Next();
//            cout<<"g_("<<v<<")->"<<g__->GetVertexData()->GetPedigreeIds()->GetVariantValue(v)<<endl;
//        }
        //cout<<"g_ edge count: "<<g__->GetNumberOfEdges()<<endl;

        return  g_;
    }

    vtkmug sort_graph (vtkmug g){

        /* This function reorders the vertex list
         * The disorder occurs when vertices are removed using RenoveVertices()
         * Order is essential for subsequent processes in the pipeline
        */

        vtkmug g_ = vtkmug::New();
        g_->DeepCopy(g);

        vtkSmartPointer<vtkEdgeListIterator> edge_it = vtkSmartPointer<vtkEdgeListIterator> ::New();
        vtkSmartPointer<vtkVertexListIterator> vertex_it = vtkSmartPointer<vtkVertexListIterator>::New();
        vtkSmartPointer<vtkVariantArray> vertex_pedigree_id = vtkSmartPointer<vtkVariantArray>::New();

        vertex_pedigree_id->SetName("vertex_pedigree_ids");

        g_->GetVertices(vertex_it);
        g_->GetEdges(edge_it);

        std::vector<string>vectex_vec;

        while(vertex_it->HasNext()){
            vtkIdType v = vertex_it->Next();
            string s = g_->GetVertexData()->GetPedigreeIds()->GetVariantValue(v).ToString();
            //cout<<"g_("<<v<<")->"<<s<<endl;
            vectex_vec.push_back(s);
            //vertex_pedigree_id->InsertNextValue(g_->GetVertexData()->GetPedigreeIds()->GetVariantValue(v));
            vertex_pedigree_id->InsertNextValue(s.c_str());
        }

        std::sort(vectex_vec.begin(),vectex_vec.end());
        vector<int> I;
        for (auto i: sort_indexes(vectex_vec)) {
          I.push_back(int(i));
        }

        //cout<<"I size: "<<I.size()<<endl;

        vtkmug g__ = vtkmug::New();
        g__->GetVertexData()->SetPedigreeIds(vertex_pedigree_id);

        g_->GetVertices(vertex_it);
        int counter = 0;

        while(vertex_it->HasNext()){
            vertex_it->Next();
            g__->AddVertex(vertex_pedigree_id->GetValue(int(I[counter])));
            //cout<<"count:"<<counter<<endl;
            counter++;
        }

        g_->GetEdges(edge_it);

        while(edge_it->HasNext()){
            vtkEdgeType edge = edge_it->Next();
            //cout<<g_->GetVertexData()->GetPedigreeIds()->GetVariantValue(edge.Source)<<", "<<g_->GetVertexData()->GetPedigreeIds()->GetVariantValue(edge.Target)<<endl;
            g__->AddEdge(g_->GetVertexData()->GetPedigreeIds()->GetVariantValue(edge.Source),g_->GetVertexData()->GetPedigreeIds()->GetVariantValue(edge.Target));
        }

        //display_graph(g__);

        //cout<<"sort g__ vertex size"<<g__->GetNumberOfVertices()<<endl;
        //cout<<"sort g__ edge size"<<g__->GetNumberOfEdges()<<endl;
        return g__;
    }

    bool still_connected(vtkmug g){

        vtkSmartPointer<vtkBoostConnectedComponents> connectedComponents = vtkSmartPointer<vtkBoostConnectedComponents>::New();
        connectedComponents->SetInputData(g);
        connectedComponents->Update(); // removes vertex with dof  = 0;

        vtkGraph* outputGraph = connectedComponents->GetOutput();
        vtkIntArray* components = vtkIntArray::SafeDownCast(outputGraph->GetVertexData()->GetArray("component"));

        std::vector<int> con_com;

        for(vtkIdType i = 0; i < components->GetNumberOfTuples(); i++){
             int val = components->GetValue(i);
             con_com.push_back(val);
         }

        int i = *std::max_element(con_com.begin(), con_com.end());

        if (i==0)
            return true;

        return false;

    }

    /*
    void display_graph(vtkmug g){
        vtkSmartPointer<vtkGraphLayoutView> graphLayoutView = vtkSmartPointer<vtkGraphLayoutView>::New();
        graphLayoutView->AddRepresentationFromInput(g);
        graphLayoutView->SetLayoutStrategy("Simple 2D");
        graphLayoutView->SetVertexLabelVisibility(true);
        vtkRenderedGraphRepresentation::SafeDownCast(graphLayoutView->GetRepresentation())->GetVertexLabelTextProperty()->SetVerticalJustificationToBottom();
        vtkRenderedGraphRepresentation::SafeDownCast(graphLayoutView->GetRepresentation())->GetVertexLabelTextProperty()->SetColor(1,0,0);
        graphLayoutView->SetVertexLabelArrayName("vertex_pedigree_ids");
        graphLayoutView->ResetCamera();
        graphLayoutView->Render();
        graphLayoutView->GetInteractor()->Start();
        graphLayoutView->SetVertexColorArrayName("Color");
        graphLayoutView->ColorVerticesOn();
    }
*/

  /*
    void showCubes(double voxelSideLen, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudVoxel ) {

        vtkSmartPointer<vtkAppendPolyData> appendFilter = vtkSmartPointer<vtkAppendPolyData>::New ();
        // Create every cubes to be displayed
        double s = voxelSideLen / 2.0;
        for (size_t i = 0; i < cloudVoxel->points.size (); i++)
        {
        double x = cloudVoxel->points[i].x;
        double y = cloudVoxel->points[i].y;
        double z = cloudVoxel->points[i].z;

        vtkSmartPointer<vtkCubeSource> wk_cubeSource = vtkSmartPointer<vtkCubeSource>::New ();

        wk_cubeSource->SetBounds (x - s, x + s, y - s, y + s, z - s, z + s);
        wk_cubeSource->Update ();

        #if VTK_MAJOR_VERSION < 6
            appendFilter->AddInput (wk_cubeSource->GetOutput ());
        #else
            appendFilter->AddInputData (wk_cubeSource->GetOutput ());
        #endif
        }

        // Remove any duplicate points
        vtkSmartPointer<vtkCleanPolyData> cleanFilter = vtkSmartPointer<vtkCleanPolyData>::New ();
        cleanFilter->SetInputConnection (appendFilter->GetOutputPort ());
        cleanFilter->Update ();

        //Create a mapper and actor
        vtkSmartPointer<vtkPolyDataMapper> multiMapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
        multiMapper->SetInputConnection (cleanFilter->GetOutputPort ());

        vtkSmartPointer<vtkActor> multiActor = vtkSmartPointer<vtkActor>::New ();
        multiActor->SetMapper (multiMapper);
        multiActor->GetProperty ()->SetColor (1.0, 1.0, 1.0);
        multiActor->GetProperty ()->SetAmbient (0.5);
        multiActor->GetProperty ()->SetLineWidth (1);
        //multiActor->GetProperty ()->EdgeVisibilityOn ();
        multiActor->GetProperty ()->SetOpacity (0.1);

        //multiActor->GetProperty()->SetEdgeVisibility(0.1);

        if (wireframe){
        multiActor->GetProperty ()->SetRepresentationToWireframe ();
        }
        else{
        multiActor->GetProperty ()->SetRepresentationToSurface ();
        }
        // Add the actor to the scene
        viz.getRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->AddActor (multiActor);
        // Render and interact
        viz.getRenderWindow ()->Render ();
    }
*/
    std::pair<bool,int> ismembertol(std::vector<tuple<float, float, float>>coor, std::tuple<float, float, float> b, float thres){
        /*
         * coor is the pcl converted into tuple Mx3 vector of tuples
         * b is the point that is one of the six possible faces of cube
         * that needs to checked for connectivity
         * thres id the threshold or tolerance for closeness
         */

        Eigen::MatrixXf response; // dynamic size
        response.resize(coor.size(),3); // resize

        std::pair<float, float> x,y,z;
        int res = 0;

        //#pragma omp parallel for
        for (size_t i = 0; i <coor.size(); i++){

            x.first  = std::get<0>(coor[i]); y.first  = std::get<1>(coor[i]);  z.first  = std::get<2>(coor[i]);
            x.second = std::get<0>(b);       y.second = std::get<1>(b);        z.second = std::get<2>(b);

            for (int j = 0; j< 3; j++){ // 3 because of three pairs
                res = 0;

                switch (j) {
                case 0:
                    res = almostEqual(x,thres);
                    //cout<<"case: "<<j<<" almostEqual("<<x.first<<", "<<x.second<<")    \t ="<<res<<endl;
                    break;
                case 1:
                    res = almostEqual(y,thres);
                    //cout<<"case: "<<j<<" almostEqual("<<y.first<<", "<<y.second<<")    \t ="<<res<<endl;
                    break;
                case 2:
                    res = almostEqual(z,thres);
                    //cout<<"case: "<<j<<" almostEqual("<<z.first<<", "<<z.second<<")    \t ="<<res<<endl;
                default:
                    break;
                }
                response(i,j)= res;
            }
        }

        Eigen::MatrixXf sum  = response.rowwise().sum();
        std::pair<bool,int> result;

        result.first = false; result.second = 0;  //default values

        #pragma omp parallel num_threads(3)
        for (Eigen::Index i = 0; i<sum.size(); ++i){
            if (sum(i) == 3){
                result.first = true;
                result.second = (int)i;
                return result;
            }
        }
        return result;
    }

    inline bool almostEqual(std::pair<float, float> X, float tolerance) {

        float A = X.first;
        float B = X.second;

        if (fabs(A-B) < tolerance)
            return true;

        return false;
    }

    Eigen::MatrixXf squareform(Eigen::MatrixXf D){

        Eigen::VectorXf N = D.rowwise().squaredNorm();
        Eigen::MatrixXf S = N.replicate(1,D.rows()) + N.transpose().replicate(D.rows(),1);

        S.noalias() -= 2. * D * D.transpose();
        S = S.array().sqrt();// This is the square form
        return S;
    }

    template <typename T> std::vector<size_t> sort_indexes(const std::vector<T> &v) {
      std::vector<size_t> idx(v.size());// initialize original index locations
      iota(idx.begin(), idx.end(), 0);
      // sort indexes based on comparing values in v
      sort(idx.begin(), idx.end(), [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});
      return idx;
    }

    void setup_target(){ // assuming that the targetPose vector has two coordinates for cut point localzation
                         // i.e. xth bud & x+1 bud 3d coordinates
                         // then we compute roll, pitch, and yaw

        //========================== Read from the nth bud and n+1th bud cloud and convert it to pose message

        cout<<"Inside setup target ()\n"<<"setting up ...."<< cut_point_cloud_n->points.size()<<" targets....."<<endl;

        geometry_msgs::Pose target_pose_1, target_pose_2;
        tf::Quaternion Q;

        budPose.clear();

        int cut_point_n_size = cut_point_cloud_n->points.size();
        int cut_point_n_minus_one_size = cut_point_cloud_n_minus_1->points.size();

        if (cut_point_n_size != cut_point_n_minus_one_size){

            ROS_ERROR("cut point vector size miss match......");
        }

        else{

            for (size_t i = 0; i <cut_point_n_size; i++){

                float pos0[3] = {cut_point_cloud_n->points[i].x, cut_point_cloud_n->points[i].y, cut_point_cloud_n->points[i].z};
                Q = tf::createQuaternionFromRPY(0 ,0 ,0);

                cout<<"point 2 x:"<<pos0[0]<<" y: "<<pos0[1]<<" z:"<<pos0[2]<<endl;
                target_pose_1.orientation.x = Q.x();
                target_pose_1.orientation.y = Q.y();
                target_pose_1.orientation.z = Q.z();
                target_pose_1.orientation.w = Q.w();

                target_pose_1.position.x = pos0[0];
                target_pose_1.position.y = pos0[1];
                target_pose_1.position.z = pos0[2];

                float pos1[3] = {cut_point_cloud_n_minus_1->points[i].x, cut_point_cloud_n_minus_1->points[i].y, cut_point_cloud_n_minus_1->points[i].z};

                target_pose_2.orientation.x = Q.x();
                target_pose_2.orientation.y = Q.y();
                target_pose_2.orientation.z = Q.z();
                target_pose_2.orientation.w = Q.w();

                target_pose_2.position.x = pos1[0];
                target_pose_2.position.y = pos1[1];
                target_pose_2.position.z = pos1[2];

                cout<<"point 1 x:"<<pos1[0]<<" y: "<<pos1[1]<<" z:"<<pos1[2]<<endl;
                budPose.push_back(target_pose_1);
                budPose.push_back(target_pose_2);
            }
        }
        cout<<"exiting bud pose function ............."<<endl;
    }

    void set_cut_point_orientation(){

        // the idea here is to not work in this frame but transfer
        // everything to world coordinate system and work there

        cout<<"inside set_cut_point_orientation ()"<<endl;
        cout<<"No. of cut points: "<<budPose.size()<<endl;

        //float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        float X1= 0.0f, Y1= 0.0f, Z1= 0.0f;
        float X2= 0.0f, Y2= 0.0f, Z2= 0.0f;

        tf::Quaternion Q;
        int counter = 0;

        targetPose.clear();

        for (size_t pose = 0; pose< (int)budPose.size()/2; pose++){

            X1 = budPose[counter].position.x;   Y1 = budPose[counter].position.y;   Z1 = budPose[counter].position.z;
            X2 = budPose[counter+1].position.x; Y2 = budPose[counter+1].position.y; Z2 = budPose[counter+1].position.z;

            Q.setEulerZYX(0, 0, 0); // orientation set to zero
            pose_msg.position.x = X1;
            pose_msg.position.y = Y1;
            pose_msg.position.z = Z1;
            pose_msg.orientation.x = Q.x();
            pose_msg.orientation.y = Q.y();
            pose_msg.orientation.z = Q.z();
            pose_msg.orientation.w = Q.w();

            poses_.poses.push_back(pose_msg);   // pushing point P1

            pose_msg.position.x = X2;
            pose_msg.position.y = Y2;
            pose_msg.position.z = Z2;
            pose_msg.orientation.x = Q.x();
            pose_msg.orientation.y = Q.y();
            pose_msg.orientation.z = Q.z();
            pose_msg.orientation.w = Q.w();

            poses_.poses.push_back(pose_msg);  // pushing point P2

            counter = counter + 2;
        }
        poses_.header.stamp = ros::Time::now();
        poses_.header.frame_id = "camera_link";
        pub_pose.publish(poses_);
    }

    void set_cut_point_orientation_abhi_(){

        cout<<"inside set_cut_point_orientation ()"<<endl;
        cout<<"No. of cut points: "<<budPose.size()<<endl;

        //float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        float X1= 0.0f, Y1= 0.0f, Z1= 0.0f;
        float X2= 0.0f, Y2= 0.0f, Z2= 0.0f;
        //float X3= 0.0f, Y3= 0.0f, Z3= 0.0f;
        float x_m = 0.0f, y_m = 0.0f, z_m = 0.0f;
        float slope_yz = 0.0f, slope_xz = 0.0f;
        float pi = 3.1415f;
        float roll = 0.0f,pitch = 0.0f, yaw = 0.0f;

        tf::Quaternion Q;
        geometry_msgs::Pose target_poseX;
        //cout<<"(int)budPose.size()/2: "<<(int)budPose.size()/3<<endl;
        int counter = 0;

        Eigen::VectorXf A(3), P1(3), P2(3), P3(3), v1(3), v2(3);
        Eigen::Vector3d V1, V2, cp; // for some reason v1.cross(v2) does not work with VectorXf so using Vector3f
        float mag1 = 0.0f, mag2 = 0.0f;
        Eigen::Vector3d yz_plane, xz_plane, YAW, ROLL;
        yz_plane << 1,0,0; xz_plane<<0,1,0;

        targetPose.clear();



        for (size_t pose = 0; pose< (int)budPose.size()/2; pose++){

            //cout<<"Pose: "<<counter<<endl;
            X1 = budPose[counter].position.x;   Y1 = budPose[counter].position.y;   Z1 = budPose[counter].position.z;
            X2 = budPose[counter+1].position.x; Y2 = budPose[counter+1].position.y; Z2 = budPose[counter+1].position.z;
            //X3 = budPose[counter+2].position.x; Y3 = budPose[counter+2].position.y; Z3 = budPose[counter+2].position.z;

            float X1_ = 0, Y1_ = 0, Z1_ = 0, X2_ = 0, Y2_ = 0, Z2_ = 0;

            X1_ = Z1;           X2_ = Z2;
            Y1_ = Y1 + 0.05313; Y2_ = Y2 + 0.05313;
            Z1_ = -Z1 + 1.2872; Z2_ = -Z2 + 1.2872;


            x_m = (float)(X1_+X2_)/2; y_m = (float)(Y1_+Y2_)/2; z_m = (float)(Z1_+Z2_)/2;
            slope_yz = (Z2_-Z1_)/(Y2_-Y1_);
            slope_xz = (Z2_-Z1_)/(X2_-X1_);

            cout<<"Point P1: ("<<X1_<<", "<<Y1_<<", "<<Z1_<<")"<<endl;
            cout<<"Point P2: ("<<X2_<<", "<<Y2_<<", "<<Z2_<<")"<<endl;
            //cout<<"Mid Point: ("<<x_m<<", "<<y_m<<", "<<z_m<<")"<<endl;
            //cout<<"slope_yz: "<<slope_yz<<" slope_xz: "<<slope_xz<<endl;

            if (Z2_>= Z1_)
                yaw = atan2((Y2_-Y1_),(X2_-X1_)) + pi/2;
            if (Z2 < Z1)
                yaw = atan2((Y2_-Y1_),(X2_-X1_)) - pi/2;

/*
            if (sign(slope_yz) >=0 && sign(slope_xz) >=0){                         //+ +
                //cout<<"Entring....... ++"<<endl;
                yaw = -(pi-acos(cp.dot(yz_plane)/mag1));
                //roll = -(pi - acos(V1.dot(xz_plane)/mag2))-yaw;
                //roll = pi/2 - acos(V1.dot(xz_plane)/mag2);
                //Q = tf::createQuaternionFromRPY(roll,pitch,yaw);
                //draw_object(x_m,y_m,z_m,Q,2,(int)pose);
                //cout<<"Exiting....... ++"<<endl;
            }

            if (sign(slope_yz) >=0 && sign(slope_xz) <0){                          //+ -
                //cout<<"Entring....... +-"<<endl;
                yaw =  acos(cp.dot(yz_plane)/mag1);
                //roll = acos(V1.dot(xz_plane)/mag2)- yaw; // branch 4 this is correct
                //Q = tf::createQuaternionFromRPY(roll,pitch,yaw);
                //draw_object(x_m,y_m,z_m,Q,2,(int)pose);
                //cout<<"Exiting....... +-"<<endl;
            }

            if (sign(slope_yz) <0 && sign(slopeQ_xz) >=0){                           //- +
                //cout<<"Entring....... -+"<<endl;
                yaw  = (pi - acos(cp.dot(yz_plane)/mag1));  // correct
                //roll = (pi/2 - acos(V1.dot(xz_plane)/mag2))-yaw; // branch 1
                //Q = tf::createQuaternionFromRPY(roll,pitch,yaw);
                //draw_object(x_m,y_m,z_m,Q,2,(int)pose);
                //cout<<"Exiting....... -+"<<endl;
            }
*/
            //cout<<"YAW Calculated......"<<endl;

            Eigen::VectorXf pm(3), pm2(3);

            pm<<x_m,y_m,z_m;
            pm2<<x_m,y_m,(z_m+0.1f);

            //cout<<"pm: "<<pm<<" pm2: "<<pm2<<endl;

            Eigen::VectorXf pm_pm2(3), pm_p2(3), pm_p1(3);
            pm_pm2<<pm2-pm;
            pm_p2<<P2-pm;
            pm_p1<<P1-pm;

            float ang_1 = acos(pm_pm2.dot(pm_p2)/(pm_pm2.norm()*pm_p2.norm()));
            float ang_2 = acos(pm_pm2.dot(pm_p1)/(pm_pm2.norm()*pm_p1.norm()));

            if (Z1_ >= Z2_ && Y1_>=Y2_)
               roll = -min(ang_1,ang_2);    //% CW positive

            if (Z1_ >= Z2_ && Y2_>=Y1_)
                roll = min(ang_1,ang_2);

            if (Z2_ >= Z1_ && Y1_>=Y2_)
                roll = min(ang_1,ang_2);

            if (Z2_ >= Z1_ && Y2_>=Y1_)
                roll = -min(ang_1,ang_2);

            cout<<"YAW: "<<yaw*180/pi<<" ROLL: "<<roll*180/pi<<endl;
            //cout<<"============================================"<<endl;


            //Q.setEulerZYX(0, 0, -25*3.1415/180);
            Q.setEulerZYX(0, 0, 0);
            pose_msg.position.x = X1;
            pose_msg.position.y = Y1;
            pose_msg.position.z = Z1;
            pose_msg.orientation.x = Q.x();
            pose_msg.orientation.y = Q.y();
            pose_msg.orientation.z = Q.z();
            pose_msg.orientation.w = Q.w();

            poses_.poses.push_back(pose_msg);
            /*
            target_poseX.position.x = x_m;  // +0.07
            target_poseX.position.y = y_m;  // -0.04
            target_poseX.position.z = z_m;

            target_poseX.orientation.x = Q.x();
            target_poseX.orientation.y = Q.y();
            target_poseX.orientation.z = Q.z();
            target_poseX.orientation.w = Q.w();

            cout<<"target_poseX.position.x: "<<target_poseX.position.x<<endl;
            cout<<"target_poseX.position.y: "<<target_poseX.position.y<<endl;
            cout<<"target_poseX.position.z: "<<target_poseX.position.z<<endl;

            cout<<"target_poseX.orientation: "<<target_poseX.orientation.x<<endl;
            cout<<"target_poseX.orientation.y: "<<target_poseX.orientation.y<<endl;
            cout<<"target_poseX.orientation.z: "<<target_poseX.orientation.z<<endl;
            cout<<"target_poseX.orientation.z: "<<target_poseX.orientation.w<<endl;
            cout<<"============================================"<<endl;

            targetPose.push_back(target_poseX);

            */
            /*
            ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("pose", 1000);

            //cout<<"here ....1."<<endl;
            geometry_msgs::PoseStamped pose_;

            pose_.pose.position.x = x_m;
            pose_.pose.position.y = y_m;
            pose_.pose.position.z = z_m;

            //tf::Quaternion Q;

            Q = tf::createQuaternionFromRPY(0, 0 , 0); //cut 0
            pose_.pose.orientation.x = Q.x();
            pose_.pose.orientation.y = Q.y();
            pose_.pose.orientation.z = Q.z();
            pose_.pose.orientation.w = Q.w();

            cout<<"pose converted ......."<<endl;

            for (int u = 0 ; u < 2; u++){

                pose_.header.frame_id = "camera_link";
                pose_.header.stamp = ros::Time::now();
                pub_pose.publish(pose_);
                sleep(1);
                cout<<"publishing pose ......"<<endl;
            }

            */

            counter = counter + 2; //3
        }

        poses_.header.stamp = ros::Time::now();
        poses_.header.frame_id = "camera_link";
        pub_pose.publish(poses_);
    }

protected:

    pcl::PointCloud<PointXYZRGB>::Ptr input_cloud;// (new PointCloud<PointXYZRGBRGB>);
    pcl::PointCloud<PointXYZRGB>::Ptr no_nan_cloud;// (new PointCloud<PointXYZRGBRGB>);
   // pcl::visualization::PCLVisualizer viz;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudVoxel;//(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr displayCloud;// (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud;// (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<PointXYZRGB>::Ptr cordon_cloud_final; // (new pcl::PointCloud<PointXYZRGB>);
    pcl::PointCloud<PointXYZRGB>::Ptr cane_cloud_final;// (new pcl::PointCloud<PointXYZRGB>);
    pcl::PointCloud<PointXYZRGB>::Ptr temp_bud_cloud;// (new pcl::PointCloud<PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr wires_fill, cordon_fill;//( new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr this_cane;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cut_point_cloud_n, cut_point_cloud_n_minus_1;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr only_cane;// (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree, kdtree_cutpoint;
    pcl::octree::OctreePointCloudPointVector<PointXYZRGB>::LeafNodeIterator it;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ransac_sample_cloud;// (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_ransac; // (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB searchPoint;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr region_growing_stop_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr MEGA_CLOUD;// (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr BUD_CLOUD;// (new pcl::PointCloud<pcl::PointXYZRGB>);

    int depth;
    custom_svd SVD;
    Ptr<SVM> svm;

    std::vector<int> only_cane_index;
    std::vector<int> cut_point_ind;
    std::vector<int> indexVector;
    std::vector<std::vector<int>> indexVV;
    std::vector<int> good_vertex_index_;
    std::vector<int> good_ped_index;
    std::vector< pcl::PointXYZRGB> pcl_stack;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    std::vector<int> inliers;
    std::vector<int> bud_indices, line_inliers;
    std::vector<string> pedg_id; // prdegree_id for graph
    std::vector<int> no_nan_index;
    std::vector<int> indices_good_buds;
    std::vector<geometry_msgs::Pose> targetPose;
    std::vector<geometry_msgs::Pose> budPose;
    vector<int>good_region_index;
    std::vector<pcl::PointIndices> cluster_indices;

    int done;
    int count_blue;
    float radius;
    float roll, pitch, yaw;
    int while_counter = 0;
    cv::Mat a_cv;
    Eigen::Matrix3f C;
    Eigen::VectorXf a;
    pcl::PointXYZRGB p1 ,p2;
    Eigen::MatrixXf::Index maxRow, maxCol;//, minRow, minCol;
    Eigen::Matrix<float, Eigen::Dynamic, 3> coor;// (row, 3);

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
    ros::Publisher pub_cut;
    ros::Publisher pub_wire;

    geometry_msgs::Pose pose_msg;
    geometry_msgs::PoseArray poses_;

};

int main(int argc, char **argv){
    ros::init(argc, argv, "cut_pose_estimator");

    ros::param::set("/terminate_loop_cut_point",0);

    get_pose gp;
    ros::spin();
    return 0;
}


