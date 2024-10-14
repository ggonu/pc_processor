#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr mCloud(new pcl::PointCloud<pcl::PointXYZ>());
ros::Publisher pub;

ros::Time lastHeaderStamp;
double sweepDuration = 0.5; // LiDAR scan sweep rate


void pointCloudCallBack(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ> cCloud;
    pcl::fromROSMsg(*msg, cCloud);
    
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        listener.waitForTransform("/Neo", "/Lidar", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("/Neo", "/Lidar", ros::Time(0), transform);

        if (mCloud->points.empty()) {
           lastHeaderStamp = msg->header.stamp;
        }

        if ((msg->header.stamp - lastHeaderStamp).toSec() < sweepDuration) {
            *mCloud += cCloud;
        } else {
            for (auto& point : mCloud->points) {
                tf::Vector3 pointLidar(point.x, point.y, point.z);
                tf::Vector3 pointTf = transform * pointLidar;
                point.x = pointTf.x();
                point.y = pointTf.y();
                point.z = pointTf.z();
            }

            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*mCloud, output);
            output.header = msg->header;
            pub.publish(output);

            mCloud->clear();
            lastHeaderStamp = msg->header.stamp;
        }
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_processor");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/point_cloud", 1, pointCloudCallBack);

    pub = nh.advertise<sensor_msgs::PointCloud2>("/processed_cloud", 1);

    ros::spin();
    return 0;
}