cmake_minimum_required(VERSION 3.8)
project(go2_onboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(go2_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# add_executable(onboard_control_node src/onboard_control_node.cpp)
# ament_target_dependencies(onboard_control_node rclcpp go2_interfaces geometry_msgs sensor_msgs std_msgs)
# install(TARGETS onboard_control_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
