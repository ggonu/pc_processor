#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/MarkerArray.h>
#include "dbscan.h"

ros::Publisher pubClusteredCloud;
ros::Publisher pubMarkers;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr cloudMsg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloudMsg, *cloud);

    float eps = 0.5;
    int minPts = 5;

    DBSCAN dbscan(cloud, eps, minPts);
    dbscan.runDbscan();

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header = cloudMsg->header;

    pubClusteredCloud.publish(output);

    visualization_msgs::MarkerArray markerArray;
    int markerId = 0;
    std::map<float, std::vector<pcl::PointXYZI>> clusters;

    for (const auto& point : cloud->points) {
        if (point.intensity != NOISE) {
            clusters[point.intensity].push_back(point);
        }
    }

    for (const auto& cluster : clusters) {
        float clusterId = cluster.first;
        const auto& points = cluster.second;

        pcl::PointXYZI centroid;
        centroid.x = 0.0;
        centroid.y = 0.0;
        centroid.z = 0.0;
        for (const auto& point : points) {
            centroid.x += point.x;
            centroid.y += point.y;
            centroid.z += point.z;
        }
        centroid.x /= points.size();
        centroid.y /= points.size();
        centroid.z /= points.size();

        visualization_msgs::Marker marker;
        marker.header.frame_id = cloudMsg->header.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "dbscan_clusters";
        marker.id = markerId++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = centroid.x;
        marker.pose.position.y = centroid.y;
        marker.pose.position.z = centroid.z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;  // Radius of the sphere
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        marker.color.r = static_cast<int>(clusterId * 50) % 255 / 255.0;
        marker.color.g = static_cast<int>(clusterId * 80) % 255 / 255.0;
        marker.color.b = static_cast<int>(clusterId * 120) % 255 / 255.0;
        marker.color.a = 1.0;

        markerArray.markers.push_back(marker);
    }

    pubMarkers.publish(markerArray);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dbscan_node");
    ros::NodeHandle nh;

    ros::Subscriber subCloud = nh.subscribe("/point_cloud", 1, cloudCallback);
    pubClusteredCloud = nh.advertise<sensor_msgs::PointCloud2>("/dbscan_clusters", 1);
    pubMarkers = nh.advertise<visualization_msgs::MarkerArray>("/dbscan_cluster_centroids", 1);

    ros::spin();
    return 0;
}