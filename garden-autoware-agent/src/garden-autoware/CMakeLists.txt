cmake_minimum_required(VERSION 2.8.3)
project(garden-autoware)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gnss
  roscpp
  std_msgs
  tf
  pcl_conversions
  carla_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

### gnss_localizer ###
add_executable(gnss_localizer
  nodes/gnss_localizer/gnss_localizer.cpp
)
target_link_libraries(gnss_localizer ${catkin_LIBRARIES})
add_dependencies(gnss_localizer ${catkin_EXPORTED_TARGETS})

### hand_over ###
add_executable(hand_over
  nodes/hand_over/hand_over.cpp
)
target_link_libraries(hand_over ${catkin_LIBRARIES})
add_dependencies(hand_over ${catkin_EXPORTED_TARGETS})
