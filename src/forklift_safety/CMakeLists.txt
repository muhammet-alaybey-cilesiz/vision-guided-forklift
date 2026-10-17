cmake_minimum_required(VERSION 3.8)
project(forklift_safety LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ROS'tan bagimsiz kapi mantigi (birim testli)
add_library(gate_logic STATIC
  include/forklift_safety/GateLogic.hpp
  src/GateLogic.cpp)
target_include_directories(gate_logic PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(cmd_vel_gate src/cmd_vel_gate_node.cpp)
target_link_libraries(cmd_vel_gate PRIVATE
  gate_logic
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS})

install(TARGETS cmd_vel_gate RUNTIME DESTINATION lib/${PROJECT_NAME})
# Komut zinciri: twist_mux + kapi yapilandirmasi ve launch (D-42)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gate_logic test/test_gate_logic.cpp)
  target_link_libraries(test_gate_logic gate_logic)
endif()

ament_package()
