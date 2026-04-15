cmake_minimum_required(VERSION 3.8)
project(go2_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

add_executable(camera_node src/camera_node.cpp)
target_include_directories(camera_node PRIVATE ${GST_INCLUDE_DIRS})
target_link_libraries(camera_node ${GST_LIBRARIES})
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
)

install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
