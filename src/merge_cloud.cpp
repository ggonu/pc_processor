#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr mCloud(new pcl::PointCloud<pcl::PointXYZ>());
ros::Publisher pub;

ros::Time lastHeaderStamp;
double sweepDuration = 0.3; // LiDAR scan sweep rate


void pointCloudCallBack(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ> cCloud;
    pcl::fromROSMsg(*msg, cCloud);

    if (mCloud->points.empty()) {
           lastHeaderStamp = msg->header.stamp;
    }

    *mCloud += cCloud;

    if ((msg->header.stamp - lastHeaderStamp).toSec() > sweepDuration) {
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*mCloud, output);
        output.header = msg->header;
        pub.publish(output);

        mCloud->clear();
        lastHeaderStamp = msg->header.stamp;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "merge_cloud");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/point_cloud", 1, pointCloudCallBack);

    pub = nh.advertise<sensor_msgs::PointCloud2>("/merged_cloud", 1);

    ros::spin();
    return 0;
}