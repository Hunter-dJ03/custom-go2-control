cmake_minimum_required(VERSION 3.8)
project(go2_description)

find_package(ament_cmake REQUIRED)

set(_vendor_go2_sdk "${CMAKE_CURRENT_SOURCE_DIR}/../vendor/go2_ros2_sdk")
set(GO2_SDK_URDF "${_vendor_go2_sdk}/go2_robot_sdk/urdf/go2.urdf")
set(GO2_SDK_MESH_DIR "${_vendor_go2_sdk}/go2_robot_sdk/dae")

if(NOT EXISTS "${GO2_SDK_URDF}")
  message(FATAL_ERROR
    "\n"
    "========================================================\n"
    "  go2_description: vendor go2_ros2_sdk URDF not found\n"
    "\n"
    "  Create a symlink to your go2_ros2_sdk checkout:\n"
    "\n"
    "    ln -sfn /path/to/unitree_ws/src/go2_ros2_sdk \\\n"
    "            ${CMAKE_CURRENT_SOURCE_DIR}/../vendor/go2_ros2_sdk\n"
    "\n"
    "  See src/vendor/README.md.\n"
    "========================================================\n"
  )
endif()

file(READ "${GO2_SDK_URDF}" _go2_urdf_text)
string(REPLACE "package://go2_robot_sdk/dae/" "package://go2_description/meshes/"
  _go2_urdf_text "${_go2_urdf_text}")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/go2.urdf.xacro" "${_go2_urdf_text}")

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/go2.urdf.xacro"
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY "${GO2_SDK_MESH_DIR}/"
  DESTINATION share/${PROJECT_NAME}/meshes
  FILES_MATCHING PATTERN "*.dae"
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
