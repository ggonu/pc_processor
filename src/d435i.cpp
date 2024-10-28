#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

#define FILTERING
#ifdef FILTERING
    #include "filtering.cpp"
    #define LEAF_SIZE 0.03f
#endif

#define TIMING
#ifdef TIMING
    #include <chrono>
#endif


ros::Publisher pub;

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // sensor_msgs::PointCloud2 -> pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXyz(new pcl::PointCloud<pcl::PointXYZ>());

    #ifdef TIMING
        auto tic = std::chrono::high_resolution_clock::now();
    #endif

    for (const auto& point : cloud->points) {
        pcl::PointXYZ p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        cloudXyz->points.push_back(p);
    }
    
    #ifdef TIMING
        auto toc = std::chrono::high_resolution_clock::now();
        std::cout << "[Converting] took " << 1e-3 * std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() << "ms" << std::endl;
    #endif

    #ifdef FILTERING
        pcl::PointCloud<pcl::PointXYZ>::Ptr fCloud(new pcl::PointCloud<pcl::PointXYZ>());
        filterVoxelGrid(LEAF_SIZE, cloudXyz, fCloud);

        std::cout << "[INFO]: Number of points: " << fCloud->size() << std::endl;
        // pcl::PointCloud -> sensor_msgs::PointCloud2
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*fCloud, output);

        output.header = msg->header;

        pub.publish(output);
    #else
        std::cout << "[INFO]: Number of points: " << cloudXyz->size() << std::endl;
        // pcl::PointCloud -> sensor_msgs::PointCloud2
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloudXyz, output);

        output.header = msg->header;

        pub.publish(output);
    #endif
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "d435i");
    ros::NodeHandle nh;

    // ros::Rate rate(10);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/d435i_cloud", 10);

    ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 10, cloud_callback);  // default: /camera/depth/color/points
    // rate.sleep();
    ros::spin();
}
