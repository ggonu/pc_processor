# pc_processor
**Point Cloud Processing package (Tested on ROS noetic)**
- This package is for **Coordinate Transform** using PointCloud2 data between camera and World Frame.
- You can change those **topic name** and **Frame name** whatever you want.

# 1. Requirements
- **Ubuntu 20.04 Focal**
- **ROS Noetic**
- **Dependencies**
  ```bash
  sudo apt-get update
  sudo apt-get install ros-noetic-pcl-ros ros-noetic-tf ros-noetic-tf2-ros ros-noetic-cv-bridge ros-noetic-pcl-ros libopencv-dev python3-opencv
  ```

# 2. Clone & Build
- Run the following commands in terminal
  ```bash
  cd catkin_ws/src
  git clone https://github.com/ggonu/pc_processor.git
  ```
- build
  ```bash
  cd ..
  catkin_make
  ```
  ```bash
  source devel/setup.bash
  ```
# 3. Run the Node
## 3-1. pc_processor
- After sourcing the setup.bash ...
  ```bash
  rosrun pc_processor pc_processor
  ```
- Subscribe (PointCloud2)
  ```ruby
  /depth_cloud
  ```
- Publish (PointCloud2)
  ```ruby
  /trans_depth_cloud
  ```
## 3-2. dbscan
- After sourcing the setup.bash ...
  ```bash
  rosrun pc_processor dbscan
  ```
- Subscribe (PointCloud2)
  ```ruby
  /point_cloud
  ```
- Publish (PointCloud2, MarkerArray)
  ```ruby
  /dbscan_clusters
  /dbscan_cluster_centroids
  ```
