#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>

// --- Define Macros --- //
                         // Uncomment Macro to Activate to using Pass Through Filter
#define PASSTH_FILTER    // Comment to Deactivate
#ifdef PASSTH_FILTER
    #include <pcl/filters/passthrough.h>
#endif

                        // Uncomment Macro to Activate to using Voxel Grid Filter
#define VOXEL_FILTER    // Comment to Deactivate
#ifdef VOXEL_FILTER
    #include <pcl/filters/voxel_grid.h>
    #define LEAF_SIZE 0.03f
#endif
                        // Uncomment Macro to Activate Debugging - check the taken time for processing
#define WITH_TIMING     // Comment to Deactivate
#ifdef WITH_TIMING
    #include <chrono>
#endif

ros::Publisher pub;


void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& iCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& fCloud) {
    #ifdef WITH_TIMING
        auto tic = std::chrono::high_resolution_clock::now();
    #endif

    #ifdef PASSTH_FILTER
        pcl::PassThrough<pcl::PointXYZ> p;
        p.setInputCloud(iCloud);
        p.setFilterFieldName("x");
        p.setFilterLimits(-3.0, 3.0);
        p.filter(*fCloud);

        p.setInputCloud(fCloud);
        p.setFilterFieldName("y");
        p.setFilterLimits(-2.0, 2.0);
        p.filter(*fCloud);
    #endif

    #ifdef VOXEL_FILTER
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(fCloud);
        vg.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
        vg.filter(*fCloud);
    #endif
    #ifdef WITH_TIMING
        auto toc = std::chrono::high_resolution_clock::now();
        std::cout << "[Filtering] took " << 1e-3 * std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() << "ms" << std::endl;
    #endif
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
    sensor_msgs::PointCloud2 cloud = *cloudMsg;

    // Convert PointCloud2 msg to PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr cCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr fCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(cloud, *cCloud);

    // Filter the point cloud
    filterPointCloud(cCloud, fCloud);
    std::cout << "[Filtering] Number of points: " << fCloud->size() << std::endl;

    // Camera coordinates -> World coordinates (apply tf manually)
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        listener.waitForTransform("/Neo", "/Camera_Pseudo_Depth", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("/Neo", "/Camera_Pseudo_Depth", ros::Time(0), transform);

        // Apply the transform to each point manually
        for (auto& point : fCloud->points) {
            tf::Vector3 pointCam(point.x, point.y, point.z);
            tf::Vector3 pointWorld = transform * pointCam;
            point.x = pointWorld.x();
            point.y = pointWorld.y();
            point.z = pointWorld.z();
        }

        // Convert to PointCloud2
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*fCloud, output);
        output.header.frame_id = "World";
        pub.publish(output);

    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pc_process");
    ros::NodeHandle nh;

    // Publish PointCloud2 topic "/trans_depth_cloud"
    pub = nh.advertise<sensor_msgs::PointCloud2>("trans_depth_cloud", 1);  // trans_depth_cloud

    // Subscribe the topic "/depth_cloud"
    ros::Subscriber sub = nh.subscribe("/d455_depth_cloud", 1, pointCloudCallback);

    ros::spin();
    return 0;
}
