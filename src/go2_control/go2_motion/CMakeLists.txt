cmake_minimum_required(VERSION 3.8)
project(go2_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_api REQUIRED)
find_package(go2_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(include)

# --- sport_client library ---
# add_library(sport_client SHARED
#   src/sport_client.cpp
# )
# ament_target_dependencies(sport_client
#   rclcpp unitree_api nlohmann_json
# )
# target_link_libraries(sport_client nlohmann_json::nlohmann_json)

# --- motion_node ---
# add_executable(motion_node src/motion_node.cpp)
# ament_target_dependencies(motion_node
#   rclcpp unitree_api go2_interfaces geometry_msgs
#   nav_msgs std_msgs rclcpp_action
# )
# target_link_libraries(motion_node sport_client)

# install(TARGETS sport_client
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# install(TARGETS motion_node
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
