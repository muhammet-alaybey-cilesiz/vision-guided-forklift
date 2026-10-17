cmake_minimum_required(VERSION 3.8)
project(forklift_gazebo)

find_package(ament_cmake REQUIRED)

# Dünya, modeller, köprü yapılandırması ve launch dosyaları
install(
  DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

# Kabul testleri: ros2 run forklift_gazebo acceptance_test.py
install(
  PROGRAMS scripts/acceptance_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
