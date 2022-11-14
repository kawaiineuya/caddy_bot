cmake_minimum_required(VERSION 3.0.3)
project(caddy_bot)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  roslib
  roslaunch
  robot_localization
)

catkin_package()
roslaunch_add_file_check(launch)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(uwb_tf src/uwb_tf.cpp)
add_executable(uwb_follow src/uwb_follow.cpp)
add_executable(camera_follow src/camera_follow.cpp)
add_executable(gps_waypoint src/gps_waypoint.cpp)
add_executable(gps_follow src/gps_follow.cpp)

target_link_libraries(uwb_tf ${catkin_LIBRARIES})
target_link_libraries(uwb_follow ${catkin_LIBRARIES})
target_link_libraries(camera_follow ${catkin_LIBRARIES})
target_link_libraries(gps_waypoint ${catkin_LIBRARIES})
target_link_libraries(gps_follow ${catkin_LIBRARIES})

install(DIRECTORY  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)