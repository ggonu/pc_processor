#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher pub;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    // Calculate transformation between LiDAR and camera Frame
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        // Convert to Lidar Frame
        listener.waitForTransform("/Lidar", "/Camera_OmniVision_OV9782_Color", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/Lidar", "/Camera_OmniVision_OV9782_Color", ros::Time(0), transform);

        for (auto& point : cloud.points) {
            tf::Vector3 point_cam(point.x, point.y, point.z);
            tf::Vector3 point_lidar = transform * point_cam;
            point.x = point_lidar.x();
            point.y = point_lidar.y();
            point.z = point_lidar.z();
        }

        // Publish PointCloud
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud, output);
        output.header.frame_id = "Lidar";
        pub.publish(output);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle nh;

    // Publish topic "trans_depth_cloud"
    pub = nh.advertise<sensor_msgs::PointCloud2>("trans_depth_cloud", 1);

    // Subscribe topic "depth_cloud"
    ros::Subscriber sub = nh.subscribe("depth_cloud", 1, pointCloudCallback);

    ros::spin();
    return 0;
}