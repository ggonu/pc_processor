cmake_minimum_required(VERSION 3.0.2)
project(pc_processor)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(dbscan_lib include/dbscan.cpp)

add_executable(pc_processor src/pc_processor.cpp)
add_executable(dbscan src/dbscan_ros.cpp)

target_link_libraries(pc_processor
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(dbscan
  dbscan_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)