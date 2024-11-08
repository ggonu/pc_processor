#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#define TIMING
#ifdef TIMING
    #include <chrono>
#endif

// #define DEBUG

namespace Filtering {
  void PassThrough(const pcl::PointCloud<pcl::PointXYZ>::Ptr& iCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& fCloud) {
    #ifdef TIMING
      auto tic = std::chrono::high_resolution_clock::now();
    #endif
    
    pcl::PassThrough<pcl::PointXYZ> p;
    // p.setInputCloud(iCloud);
    // p.setFilterFieldName("x");
    // p.setFilterLimits(-3.0, 3.0);
    // p.filter(*fCloud);

    // p.setInputCloud(fCloud);
    // p.setFilterFieldName("y");
    // p.setFilterLimits(-2.0, 2.0);
    // p.filter(*fCloud);

    p.setInputCloud(iCloud);
    p.setFilterFieldName("z");
    p.setFilterLimits(0.0, 1.6);
    p.filter(*fCloud);

    #ifdef TIMING
      auto toc = std::chrono::high_resolution_clock::now();
      std::cout << "[Filtering] took " << 1e-3 * std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() << "ms" << std::endl;
    #endif
  }

  void VoxelGrid(auto leafSize, const pcl::PointCloud<pcl::PointXYZ>::Ptr& iCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& fCloud) {
    #ifdef TIMING
      auto tic = std::chrono::high_resolution_clock::now();
    #endif
    
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(iCloud);
    vg.setLeafSize(leafSize, leafSize, leafSize);
    vg.filter(*fCloud);

    #ifdef TIMING
      auto toc = std::chrono::high_resolution_clock::now();
      std::cout << "[VoxelGrid] took " << 1e-3 * std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() << "ms" << std::endl;
    #endif
  }

#ifdef DEBUG
  void FindGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr& iCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& gCloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr fCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(1.8);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);

    seg.setInputCloud(iCloud);
    seg.segment(*inliers, *coeff);

    if (inliers->indices.empty()) {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        return;
    }
    
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud()
  }
#endif

} // namespace Filtering