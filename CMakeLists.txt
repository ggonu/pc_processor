cmake_minimum_required(VERSION 3.0.2)
project(pc_processor)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pc_processor src/pc_processor.cpp)
add_executable(transform_lidar_camera src/transform_lidar_camera.cpp)

target_link_libraries(pc_processor
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(transform_lidar_camera
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)