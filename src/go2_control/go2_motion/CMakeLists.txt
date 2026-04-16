cmake_minimum_required(VERSION 3.8)
project(go2_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_api REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(cmd_vel_to_sport_node src/cmd_vel_to_sport_node.cpp)
ament_target_dependencies(cmd_vel_to_sport_node
  rclcpp unitree_api geometry_msgs nlohmann_json
)
target_link_libraries(cmd_vel_to_sport_node nlohmann_json::nlohmann_json)

install(TARGETS cmd_vel_to_sport_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
