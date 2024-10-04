#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

#include <thread>
#include "dbscan.h"

ros::Publisher pubClusteredCloud;
ros::Publisher pubMarkers;

void runDbscanThread(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float eps, int minPts) {
    DBSCAN dbscan(cloud, eps, minPts);
    dbscan.runDbscan();
}

void createClusterBoundaryMarkers(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster, int clusterId, const std_msgs::Header& header, visualization_msgs::MarkerArray& markerArray) {
    pcl::ConvexHull<pcl::PointXYZI> convexHull;
    pcl::PointCloud<pcl::PointXYZI>::Ptr hullPoints(new pcl::PointCloud<pcl::PointXYZI>());
    convexHull.setInputCloud(cluster);
    convexHull.reconstruct(*hullPoints);

    visualization_msgs::Marker lineStrip;
    lineStrip.header = header;
    lineStrip.ns = "cluster_boundaries";
    lineStrip.id = clusterId;
    lineStrip.type = visualization_msgs::Marker::LINE_STRIP;
    lineStrip.action = visualization_msgs::Marker::ADD;
    lineStrip.pose.orientation.w = 1.0;
    lineStrip.scale.x = 0.05;

    lineStrip.color.r = static_cast<int>(clusterId * 50) % 255 / 255.0;
    lineStrip.color.g = static_cast<int>(clusterId * 80) % 255 / 255.0;
    lineStrip.color.b = static_cast<int>(clusterId * 120) % 255 / 255.0;
    lineStrip.color.a = 1.0;

    for (const auto& point : hullPoints->points) {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        lineStrip.points.push_back(p);
    }

    if (!hullPoints->points.empty()) {
        geometry_msgs::Point p;
        p.x = hullPoints->points.front().x;
        p.y = hullPoints->points.front().y;
        p.z = hullPoints->points.front().z;
        lineStrip.points.push_back(p);
    }

    markerArray.markers.push_back(lineStrip);
}

// Optimize filtering using PassThrough filter
void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filteredCloud) {
    // PassThrough filter for distance
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(inputCloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-3.0, 3.0); // Adjust the range for the robot's forward direction
    pass.filter(*filteredCloud);
    
    pass.setInputCloud(filteredCloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-3.0, 3.0); // Adjust the range for side-to-side direction
    pass.filter(*filteredCloud);
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloudMsg, *cCloud);

    // Apply down-sampling using VoxelGrid filter (before normal estimation)
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cCloud);
    vg.setLeafSize(0.2f, 0.2f, 0.2f); // Adjust for more aggressive down-sampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>());
    vg.filter(*filteredCloud);

    // Apply angle and distance filtering using PassThrough
    pcl::PointCloud<pcl::PointXYZ>::Ptr rangeFilteredCloud(new pcl::PointCloud<pcl::PointXYZ>());
    filterPointCloud(filteredCloud, rangeFilteredCloud);

    // Estimate surface normals (after down-sampling)
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(rangeFilteredCloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setKSearch(30);  // Reduce number of neighbors for faster computation
    ne.compute(*normals);

    // Filter points based on normals
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (size_t i = 0; i < normals->points.size(); ++i) {
        if (std::abs(normals->points[i].normal_z) > 0.9)
            inliers->indices.push_back(i);
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(rangeFilteredCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZ>());
    extract.filter(*planeCloud);

    // Convert to XYZI format
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for (const auto& point : planeCloud->points) {
        pcl::PointXYZI cvtCloud;
        cvtCloud.x = point.x;
        cvtCloud.y = point.y;
        cvtCloud.z = point.z;
        cvtCloud.intensity = UNCLASSIFIED;
        cloud->points.push_back(cvtCloud);
    }

    // *** Set parameters *** //
    float eps = 0.5;  // Adjust for better clustering
    int minPts = 6;

    // Run DBSCAN
    ros::Time startTime = ros::Time::now();
    std::thread dbscanThread(runDbscanThread, cloud, eps, minPts);
    dbscanThread.join();
    ros::Time endTime = ros::Time::now();
    ROS_INFO("[INFO]: DBSCAN took %f seconds", (endTime - startTime).toSec());

    // Publish the segmented point cloud
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header = cloudMsg->header;
    pubClusteredCloud.publish(output);

    // Create markers for visualization
    visualization_msgs::MarkerArray markerArray;
    int clusterId = 0;
    std::map<float, std::vector<pcl::PointXYZI>> clusters;

    // Collect clusters
    for (const auto& point : cloud->points) {
        if (point.intensity != NOISE) {
            clusters[point.intensity].push_back(point);
        }
    }

    ROS_INFO("[DEBUG]: Number of clusters found: %lu", clusters.size());

    // Visualize each cluster as a marker
    for (const auto& cluster : clusters) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
        clusterCloud->points.insert(clusterCloud->points.end(), cluster.second.begin(), cluster.second.end());
        createClusterBoundaryMarkers(clusterCloud, clusterId++, cloudMsg->header, markerArray);
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
