cmake_minimum_required(VERSION 3.8)
project(go2_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(go2_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# add_executable(lidar_processor_node src/lidar_processor_node.cpp)
# ament_target_dependencies(lidar_processor_node
#   rclcpp sensor_msgs pcl_conversions pcl_ros laser_geometry tf2_ros
# )

# add_executable(camera_processor_node src/camera_processor_node.cpp)
# ament_target_dependencies(camera_processor_node
#   rclcpp sensor_msgs cv_bridge image_transport
# )

# add_executable(sensor_tf_node src/sensor_tf_node.cpp)
# ament_target_dependencies(sensor_tf_node
#   rclcpp tf2_ros tf2_geometry_msgs
# )

# install(TARGETS
#   lidar_processor_node
#   camera_processor_node
#   sensor_tf_node
#   DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
