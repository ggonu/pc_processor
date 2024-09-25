#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher pub;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Convert PointCloud2 msg to PCL format
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    // Camera coordinates to World coordinates
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        listener.waitForTransform("/World", "/Camera_OmniVision_OV9782_Color", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/World", "/Camera_OmniVision_OV9782_Color", ros::Time(0), transform);

        for (auto& point : cloud.points) {
            // Camera coordinate points to World coordinate points
            tf::Vector3 point_cam(point.x, point.y, point.z);
            tf::Vector3 point_world = transform * point_cam;
            point.x = point_world.x();
            point.y = point_world.y();
            point.z = point_world.z();
            // ROS_INFO("World X: %f, Y: %f, Z: %f", point_world.x(), point_world.y(), point_world.z());
        }

        // Convert converted PointCloud to PointCloud2
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(cloud, output);
        output.header.frame_id = "World";   // Pub to World coordinates
        pub.publish(output);                // For new topic

    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pc_process");
    ros::NodeHandle nh;

    // Publish PointCloud2 topic "trans_depth_cloud"
    pub = nh.advertise<sensor_msgs::PointCloud2>("trans_depth_cloud", 1);

    // Subscribe the topic "depth_cloud"
    ros::Subscriber sub = nh.subscribe("/depth_cloud", 1, pointCloudCallback);

    ros::spin();
    return 0;
}
