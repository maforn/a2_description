cmake_minimum_required(VERSION 3.16)
project(a2_description)

set (
  DEPENDENCY_LIST
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  sensor_msgs
  rosgraph_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
  tf2_sensor_msgs
)


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(a2_bridge src/a2_bridge.cpp)
ament_target_dependencies(a2_bridge ${DEPENDENCY_LIST})

install(TARGETS
  a2_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install the a2 directory to the share folder
install(DIRECTORY 
  urdf 
  mjcf 
  meshes
  rviz
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()