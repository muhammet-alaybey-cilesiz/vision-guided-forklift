cmake_minimum_required(VERSION 3.8)
project(forklift_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# Algı → otonomi → GUI arayüzü (docs/PLAN_CV_AUTONOMY.md §4.2)
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/StationSign.msg
  msg/StationSignArray.msg
  msg/Lane.msg
  msg/LaneArray.msg
  msg/BayTarget.msg
  msg/BayTargetArray.msg
  msg/Parcel.msg
  msg/ParcelArray.msg
  msg/LoadState.msg
  msg/AutonomyState.msg
  srv/SetMode.srv
  DEPENDENCIES std_msgs geometry_msgs vision_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
