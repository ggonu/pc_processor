#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#define TIMING
#ifdef TIMING
    #include <chrono>
#endif

void filterPassThrough(const pcl::PointCloud<pcl::PointXYZ>::Ptr& iCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& fCloud) {
    #ifdef TIMING
        auto tic = std::chrono::high_resolution_clock::now();
    #endif
    
    pcl::PassThrough<pcl::PointXYZ> p;
    p.setInputCloud(iCloud);
    p.setFilterFieldName("x");
    p.setFilterLimits(-3.0, 3.0);
    p.filter(*fCloud);

    p.setInputCloud(fCloud);
    p.setFilterFieldName("y");
    p.setFilterLimits(-2.0, 2.0);
    p.filter(*fCloud);

    #ifdef TIMING
        auto toc = std::chrono::high_resolution_clock::now();
        std::cout << "[Filtering] took " << 1e-3 * std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() << "ms" << std::endl;
    #endif
}

void FilterVoxelGrid(float leafSize, const pcl::PointCloud<pcl::PointXYZ>::Ptr& iCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& fCloud) {
    #ifdef TIMING
        auto tic = std::chrono::high_resolution_clock::now();
    #endif
    
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(fCloud);
    vg.setLeafSize(leafSize, leafSize, leafSize);
    vg.filter(*fCloud);

    #ifdef TIMING
        auto toc = std::chrono::high_resolution_clock::now();
        std::cout << "[VoxelGrid] took " << 1e-3 * std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() << "ms" << std::endl;
    #endif
}