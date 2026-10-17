cmake_minimum_required(VERSION 3.8)
project(forklift_description)

find_package(ament_cmake REQUIRED)

# Yalnızca veri paketi: tanım, parametreler, launch ve RViz yapılandırması
install(
  DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
