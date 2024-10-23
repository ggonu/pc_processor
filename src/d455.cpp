#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


ros::Publisher pub;

// Parameters (Intel RealSense D455)
float k1 = 0.00245;
// float k2 = 0.0;
// float k3 = 0.0;
float p0 = -0.00037;
float p1 = -0.00074;

float fx  = 1.9299999475479126;// * 1936;   // Focal length
float fy  = 1.9299999475479126;// * 1216;   // Focal length
float cx = 970.94244;   // Optical center
float cy = 600.37482;   // Optical center


void unDistortion(float& x, float& y, float& z) {
    x /= z;
    y /= z;

    float r2 = x*x + y*y;

    // Radial distortion
    float radialDistort = 1 + k1*r2; // + k2*r2*r2 + k3*r2*r2*r2;
    x *= radialDistort;
    y *= radialDistort;

    // Tangential distortion
    float xTangent = 2*p0*x*y + p1*(r2 + 2*x*x);
    float yTangent = p0*(r2 + 2*y*y) + 2*p1*x*y;

    x += xTangent;
    y += yTangent;
}

void d455Callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloudMsg, *cCloud);

    for (auto& point : cCloud->points) {
        if (point.z > 0) {
            float x = point.x;
            float y = point.y;
            float z = point.z;
            
            ROS_INFO("Original: x=%.3f, y=%.3f, z=%.3f", x, y, z);
            unDistortion(x, y, z);
            ROS_INFO("Corrected: x=%.3f, y=%.3f, z=%.3f", x, y, z);

            point.x = x;
            point.y = y;
            point.z = z;
        }
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cCloud, output);
    output.header = cloudMsg->header;

    pub.publish(output);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "d455");
    ros::NodeHandle nh;

    // Publish PointCloud2 topic "/trans_depth_cloud"
    pub = nh.advertise<sensor_msgs::PointCloud2>("d455_depth_cloud", 1);

    // Subscribe the topic "/depth_cloud"
    ros::Subscriber sub = nh.subscribe("/depth_cloud", 1, d455Callback);

    ros::spin();
    return 0;
}
