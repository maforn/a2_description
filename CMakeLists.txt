cmake_minimum_required(VERSION 3.16)
project(a2_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  mjcf
  meshes
  rviz
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
