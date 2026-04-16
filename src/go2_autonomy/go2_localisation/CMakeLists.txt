cmake_minimum_required(VERSION 3.8)
project(go2_localisation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(map_odom_tf_node
  src/map_odom_tf_node.cpp
)
ament_target_dependencies(map_odom_tf_node
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)

add_executable(odom_path_history_node
  src/odom_path_history_node.cpp
)
ament_target_dependencies(odom_path_history_node
  geometry_msgs
  nav_msgs
  rclcpp
)

install(TARGETS map_odom_tf_node odom_path_history_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
