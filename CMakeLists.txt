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
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(dbscan_lib include/dbscan.cpp)
add_library(filtering src/filtering.cpp)

add_executable(pc_processor src/pc_processor.cpp)
add_executable(dbscan src/dbscan_ros.cpp)
add_executable(merge_cloud src/merge_cloud.cpp)
add_executable(d455 src/d455.cpp)
add_executable(d435i src/d435i.cpp)

target_link_libraries(pc_processor
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(dbscan
  dbscan_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(merge_cloud
  ${catkin_LIBRARIES}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(d455
  ${catkin_LIBRARIES}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(d435i
  ${catkin_LIBRARIES}
  ${PCL_INCLUDE_DIRS}
  filtering
)