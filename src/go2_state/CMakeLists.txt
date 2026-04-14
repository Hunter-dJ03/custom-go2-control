cmake_minimum_required(VERSION 3.8)
project(go2_state)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(go2_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# add_executable(state_aggregator_node src/state_aggregator_node.cpp)
# ament_target_dependencies(state_aggregator_node
#   rclcpp go2_interfaces nav_msgs sensor_msgs diagnostic_msgs std_msgs
# )

# install(TARGETS
#   state_aggregator_node
#   DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
