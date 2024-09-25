# pc_processor
**Point Cloud Processing package (Tested on ROS noetic)**
- You can change those **topic name** and **Frame name** whatever you want.

# 1. Requirements
- Ubuntu 20.04 focal
- ROS Noetic
- Dependencies
  ```bash
  sudo apt-get update
  sudo apt-get install ros-noetic-pcl-ros ros-noetic-tf ros-noetic-tf2-ros ros-noetic-cv-bridge ros-noetic-pcl-ros libopencv-dev python3-opencv
  ```

# 2. Install & Build
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
- After sourcing the setup.bash ...
  ```bash
  rosrun pc_processor pc_processor
  ```
