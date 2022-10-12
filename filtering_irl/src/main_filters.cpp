////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ROS core
#include <ros/ros.h>

// PCL includes
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

//Conversion
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

//pcl_ros
#include <pcl_ros/point_cloud.h>

//DROR filter requirements
#include <filtering_irl/custom_filter.h>
#include <filtering_irl/custom_filter.hpp>
#include <filtering_irl/LIOR.h>
#include <filtering_irl/LIOR.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <chrono>


using namespace std;

/**
\author Radu Bogdan Rusu
@b pointcloud_filter is a simple node that retrieves a ROS point cloud message and saves it to disk into a PCD (Point
Cloud Data) file format.
**/
class main_filters {
    typedef pcl::PointXYZI PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

protected:
    ros::NodeHandle nh;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher pub_fil;
    ros::Publisher pub_unfil;
private:
    pcl::CustomFilter<PointT> cust_filter;
    pcl::LIOR<PointT> lior;
    pcl::KdTreeFLANN<PointT> kdFLANN;
public:    
    double max_intens;
    double angle_res;
    double const_multi;
    int min_nbrs;
    int max_threads;
//public:
    string cloud_topic;

    ////////////////////////////////////////////////////////////////////////////////
    main_filters () {

        nh = ros::NodeHandle("~");

        //cloud_topic = "/input";
        //set_params();
        //cout << cloud_topic << '\n' << max_threads << '\n';                
        cloud_topic = "/os_cloud_node/points";
        cust_filter.setSearchMethod(kdFLANN);
        lior.setSearchMethod(kdFLANN);

        // Ouster 0 128
        // 0.4 angle res, covers 2 points
        // Multiplier = 5, covers 5*2 = 10
        //cust_filter.setMultiplier(0.4, 5.0);
        //cust_filter.setMinNeighborsInRadius(5);

        /*
        * Ouster 2 128 : Visvulizer https://www.desmos.com/calculator/9yqfbxvtoq
        * Horiz = 360 d / 1024 = 0.35156
        * Verti = 22.5d / 128  = 0.17578
        * ~0.36 angle res, covers 6 points
        * Multiplier = 2.25, covers 6*2.25 = ~13
        * 
        * Execution 30ms with param:
        * angle_res = max(360.0/1024.0, 22.5/128.0) + 0.05;
        * setMultiplier(angle_res, 4.0);
        * setMinNeighborsInRadius(32);
        * 
        * Execution 30ms with param:
        * angle_res = max(360.0/1024.0, 22.5/128.0) + 0.05;
        * setMultiplier(angle_res, 4.0);
        * setMinNeighborsInRadius(32);
        */ 
        angle_res = max(360.0/1024.0, 22.5/128.0) + 0.05;
        cust_filter.setMultiplier(angle_res, 4.0);
        cust_filter.setMinNeighborsInRadius(32);

        lior.setDetection_range(71.235);
        lior.setMinNeighborsInRadius(3);

        //Subscribing
        sub_ = nh.subscribe(cloud_topic, 1, &main_filters::cloud_cb, this);
        // ********** 
        ROS_INFO ("Listening for incoming data on topic %s", nh.resolveName(cloud_topic).c_str());

        //Publishing
        pub_fil = nh.advertise<PointCloud>("/filtered_pcd", 1);
        pub_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_sensor_msg", 1);
        //pub_unfil = nh.advertise<PointCloud>("/unfiltered_pcd", 1);
    }

    ~main_filters() {}

    void set_params() {
        /*
        * Basic Template
        * 
        * string check;
        * nh.getParam("param", check);
        * rosrun ... _param:=string_value
        * 
        */ 

        if(nh.hasParam("in")){
            nh.getParam("in", cloud_topic);
        }
        if(nh.hasParam("angle_res")){
            nh.getParam("angle_res", angle_res);
        }
        if(nh.hasParam("const_multi")){
            nh.getParam("const_multi", const_multi);
        }
        if(nh.hasParam("min_neighbors")){
            nh.getParam("min_neighbors", min_nbrs);
        }
        if(nh.hasParam("max_intensity")){
            nh.getParam("max_intensity", max_intens);
        }
        if(nh.hasParam("max_threads")){
            nh.param<int>("max_threads", max_threads, 1);
        }

        ROS_INFO(" cloud_topic - %s \n angle_res   - %f \n const_multi - %f \n min_nbrs    - %d \n max_intens  - %f \n max_threads - %d", cloud_topic.c_str(), angle_res, const_multi, min_nbrs, max_intens, max_threads);
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb (const sensor_msgs::PointCloud2::ConstPtr& input) {
    //void cloud_cb (const PointCloud::ConstPtr& input) {
        if ((input->width * input->height) == 0)
            return;

        /********** ROS_INFO ("Received %d data points in frame %s with the following fields: %s",
                  (int) input->width * input->height,
                  input->header.frame_id.c_str(),
                  pcl::getFieldsList(*input).c_str()); */

        //Converting Message to data
        //PointCloud2 Sensor msgs to PointCloud

        PointCloud::Ptr unfilter_cloud (new PointCloud);
        PointCloud::Ptr filter_cloud (new PointCloud);

        // ********** ROS_INFO("Converting sensor_msg to pcl PointCloud2---");
        pcl::fromROSMsg(*input,*unfilter_cloud);

        
        //pub_unfil.publish(*unfilter_cloud);
        auto unfil_points = unfilter_cloud->width * unfilter_cloud->height; 

        // ********** ROS_INFO("Starting Filter Algo---");
        // ********** ROS_INFO("Setting Input Cloud---");
        cust_filter.setInputCloud(unfilter_cloud);
        lior.setInputCloud(unfilter_cloud);

        // ********** ROS_INFO("Applying Filter---");
        auto start = chrono::high_resolution_clock::now();

//        cust_filter.applyDROR_omp(*filter_cloud);
//        cust_filter.applyDROR_omp_nan(*filter_cloud);
        //cust_filter.applyDROR_omp_v2(*filter_cloud);
        lior.applyFilter(*filter_cloud);

        auto stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
        //ROS_INFO("Execution Time: %ld", duration.count());

        // ********** ROS_INFO("Filter Algo Done!!");
        
        auto fil_points = (*filter_cloud).width * (*filter_cloud).height;
        //ROS_INFO("Points Removed     : %d", unfil_points - fil_points);
        //ROS_INFO("Execution TIme : %ld \t POints removed : %d", duration.count(), unfil_points - fil_points);
        cout << "Time: " << duration.count() << " | Delta: " << (unfil_points - fil_points) << '\n';

        filter_cloud->header = unfilter_cloud->header;
        filter_cloud->is_dense = false;

        // ********** ROS_INFO("Publishing Filtered Cloud---");
        pub_fil.publish(*filter_cloud);

        sensor_msgs::PointCloud2 filter_sensor_msg;
        //pcl_conversions::fromPCL(*filter_cloud, filter_sensor_msg);
        pcl::toROSMsg(*filter_cloud, filter_sensor_msg);
        pub_.publish(filter_sensor_msg);

        // ********** ROS_INFO("Published");
    }
};

/* ---[ */
int main (int argc, char** argv) {
    //ros::init(argc, argv, "pointcloud_filter", ros::init_options::AnonymousName);
    ros::init(argc, argv, "node_name");
    main_filters pf;

    // Single Thread
    ros::spin();

    // Multi thread ***************
    //ros::MultiThreadedSpinner spinner(4);
    //spinner.spin();
    return (0);
}
/* ]--- */