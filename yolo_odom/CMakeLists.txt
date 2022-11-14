cmake_minimum_required(VERSION 3.0.2)
project(yolo_odom)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  darknet_ros_msgs
  tf
  image_geometry
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES yolo_odom
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom src/odom.cpp)

target_link_libraries(odom ${catkin_LIBRARIES})

find_package(realsense2 REQUIRED)
target_link_libraries(odom ${realsense2_LIBRARY})