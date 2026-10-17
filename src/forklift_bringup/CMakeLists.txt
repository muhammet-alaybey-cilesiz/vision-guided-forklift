cmake_minimum_required(VERSION 3.8)
project(forklift_bringup)

find_package(ament_cmake REQUIRED)

# Yalniz launch dosyasi; komut zinciri forklift_safety'de (D-42)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
