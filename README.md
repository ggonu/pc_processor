# pc_processor
**Point Cloud Processing package (Tested on ROS noetic)**
- This package is for **Coordinate Transform** using PointCloud2 data between camera and World Frame.
- You can change those **topic name** and **Frame name** whatever you want.
- Using the package with **Isaac Sim**
  - If you want to interact with **Isaac Sim**, must lauch the **ROS Bridge Extension** when you launch the simulation before.
    ![image](https://github.com/user-attachments/assets/2d073498-1c32-479c-b94b-8df47d21ec55)


# 1. Requirements
- **Ubuntu 20.04 Focal**
- **ROS Noetic**
- **Dependencies**
  ```bash
  sudo apt-get update
  sudo apt-get install ros-noetic-pcl-ros ros-noetic-tf ros-noetic-tf2-ros ros-noetic-cv-bridge ros-noetic-pcl-ros libopencv-dev python3-opencv
  ```
  - Or, you can also use `rosdep`
- **Realsense D435i**
  - Add info later...
# 2. Clone & Build
- Run the following commands in terminal
- If you don't have any workspaces to use this package...
  ```bash
  mkdir -p catkin_ws/src
  ```
- then,
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
**!!** Don't forget **run roscore** before run those nodes. **!!**
## 3-1. pc_processor
- After sourcing the setup.bash ...
  ```bash
  rosrun pc_processor pc_processor
  ```
- Subscribe (PointCloud2)
  ```bash
  /depth_cloud
  ```
- Publish (PointCloud2)
  ```bash
  /trans_depth_cloud
  ```
## 3-2. dbscan
**Do not use this node. (2024-10-04)**
- After sourcing the setup.bash ...
  ```bash
  rosrun pc_processor dbscan
  ```
- Subscribe (PointCloud2)
  ```bash
  /point_cloud
  ```
- Publish (PointCloud2, MarkerArray)
  ```bash
  /dbscan_clusters
  /dbscan_cluster_centroids
  ```
- For example, you can see MarkerArray that visualize the boundary of cluster.
  ![24-10-04_dbscan-test_3](https://github.com/user-attachments/assets/d1bd6ad4-0f75-4a0e-8a91-50cefb33b554)

## 3-3. merge_cloud
- After sourcing the setup.bash ...
  ```bash
  rosrun pc_processor merge_cloud
  ```
- Subscribe (PointCloud2)
  ```bash
  /point_cloud
  ```
- Publish (PointCloud2)
  ```bash
  /merged_cloud
  ```
