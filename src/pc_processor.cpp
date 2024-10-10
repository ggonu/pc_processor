#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

ros::Publisher pub;

// Distortion parameters (Intel RealSense D455)
static double k1 = 0.00245;  // Radial distortion coefficient 0.00245
static double k2 = -0.00037; // Radial distortion coefficient -0.00037
static double p1 = -0.00074; // Tangential distortion coefficient -0.00074
static double p2 = -0.00058; // Tangential distortion coefficient -0.00058

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
    // Convert PointCloud2 msg to PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloudMsg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr correctedCloud(new pcl::PointCloud<pcl::PointXYZ>());

    // Apply distortion correction
    for (const auto& point : cloud->points) {
        if (point.z > 0) {
            // Compute normalized coordinates
            double x_normalized = point.x / point.z;
            double y_normalized = point.y / point.z;

            // Compute radial distance from the optical center
            double r2 = x_normalized * x_normalized + y_normalized * y_normalized;
            double radialDistortion = 1 + k1 * r2 + k2 * r2 * r2;

            // Correct x and y coordinates with radial and tangential distortion
            double x_corrected = x_normalized * radialDistortion + (2 * p1 * x_normalized * y_normalized + p2 * (r2 + 2 * x_normalized * x_normalized));
            double y_corrected = y_normalized * radialDistortion + (p1 * (r2 + 2 * y_normalized * y_normalized) + 2 * p2 * x_normalized * y_normalized);

            // Rescale corrected coordinates back to 3D
            x_corrected *= point.z;
            y_corrected *= point.z;

            correctedCloud->points.push_back(pcl::PointXYZ(x_corrected, y_corrected, point.z));
        }
    }

    // Camera coordinates -> World coordinates (apply tf manually)
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        listener.waitForTransform("/World", "/Camera_OmniVision_OV9782_Color", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("/World", "/Camera_OmniVision_OV9782_Color", ros::Time(0), transform);

        // Apply the transform to each point manually
        for (auto& point : correctedCloud->points) {
            tf::Vector3 point_cam(point.x, point.y, point.z);
            tf::Vector3 point_world = transform * point_cam;
            point.x = point_world.x();
            point.y = point_world.y();
            point.z = point_world.z();
        }

        // Convert to PointCloud2
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*correctedCloud, output);
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
    pub = nh.advertise<sensor_msgs::PointCloud2>("trans_depth_cloud", 1);

    // Subscribe the topic "/depth_cloud"
    ros::Subscriber sub = nh.subscribe("/depth_cloud", 1, pointCloudCallback);

    ros::spin();
    return 0;
}
