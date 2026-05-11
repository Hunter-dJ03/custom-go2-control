cmake_minimum_required(VERSION 3.8)
project(go2_lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(lidar_calibration_node
  src/lidar_calibration_node.cpp
)
ament_target_dependencies(lidar_calibration_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
)
target_compile_features(lidar_calibration_node PUBLIC cxx_std_17)

install(TARGETS lidar_calibration_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
