cmake_minimum_required(VERSION 3.8)
project(go2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(go2_interfaces REQUIRED)

add_executable(go2_bridge_node
  src/go2_bridge_node.cpp
)
ament_target_dependencies(go2_bridge_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  unitree_go
  unitree_api
  go2_interfaces
)

install(TARGETS go2_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
