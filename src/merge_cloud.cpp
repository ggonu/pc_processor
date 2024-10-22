#include <ros/ros.h>


#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>


// --- Define Macros --- //
                        // Uncomment Macro to Activate to using Voxel Grid Filter
#define VOXEL_FILTER    // Commnet to Deactivate
#ifdef VOXEL_FILTER
#include <pcl/filters/voxel_grid.h>
#define LEAF_SIZE 0.03f
#endif
                        // Uncommnet Macro to Activate to Debugging - check the taken time for processing
#define WITH_TIMING     // Comment to Deactivate
#ifdef WITH_TIMING
#include <chrono>
#endif



pcl::PointCloud<pcl::PointXYZ>::Ptr mCloud(new pcl::PointCloud<pcl::PointXYZ>());
ros::Publisher pub;

ros::Time lastHeaderStamp;
double sweepDuration = 0.2; // LiDAR scan sweep rate

void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& iCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& fCloud) {
    #ifdef WITH_TIMING
        auto tic = std::chrono::high_resolution_clock::now();
    #endif

    pcl::PassThrough<pcl::PointXYZ> p;
    p.setInputCloud(iCloud);
    p.setFilterFieldName("x");
    p.setFilterLimits(0.0, 6.0);
    p.filter(*fCloud);

    p.setInputCloud(iCloud);
    p.setFilterFieldName("y");
    p.setFilterLimits(-3.0, 3.0);
    p.filter(*fCloud);

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

void pointCloudCallBack(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ> cCloud;
    pcl::fromROSMsg(*msg, cCloud);

    if (mCloud->points.empty()) {
           lastHeaderStamp = msg->header.stamp;
    }

    *mCloud += cCloud;

    if ((msg->header.stamp - lastHeaderStamp).toSec() > sweepDuration) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr fCloud(new pcl::PointCloud<pcl::PointXYZ>());
        sensor_msgs::PointCloud2 output;
        filterPointCloud(mCloud, fCloud);
        std::cout << "[Filtering] Number of points: " << fCloud->size() << std::endl;
        pcl::toROSMsg(*fCloud, output);
        output.header = msg->header;
        pub.publish(output);

        mCloud->clear();
        lastHeaderStamp = msg->header.stamp;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "merge_cloud");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/point_cloud", 100, pointCloudCallBack);  // default: "/point_cloud"

    pub = nh.advertise<sensor_msgs::PointCloud2>("/merged_cloud", 100);

    ros::spin();
    return 0;
}