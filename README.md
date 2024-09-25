# pc_processor
**Point Cloud Processing package (Tested on ROS noetic)**
---

# 1. Requirements
- Ubuntu 20.04 focal
- ROS Noetic
- Dependencies
  '''
  sudo apt-get update
  sudo apt-get install ros-noetic-pcl-ros ros-noetic-tf ros-noetic-tf2-ros ros-noetic-cv-bridge ros-noetic-pcl-ros libopencv-dev python3-opencv
  '''

## 1-1. installation
- Run the following commands in terminal
  '''
  cd catkin_ws/src
  git clone https://github.com/ggonu/pc_processor.git
  '''
- build
  '''
  cd ..
  catkin_make
  '''
  '''
  source devel/setup.bash
  '''
