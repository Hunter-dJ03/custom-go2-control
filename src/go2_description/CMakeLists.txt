cmake_minimum_required(VERSION 3.8)
project(go2_description)

find_package(ament_cmake REQUIRED)

# URDF and Collada meshes are stored in-tree under urdf/ and meshes/
# (origin: community go2_ros2_sdk, BSD license). Mesh URLs in go2.urdf are
# already rewritten to package://go2_description/meshes/.
set(_go2_urdf "${CMAKE_CURRENT_SOURCE_DIR}/urdf/go2.urdf")
if(NOT EXISTS "${_go2_urdf}")
  message(FATAL_ERROR
    "\n"
    "========================================================\n"
    "  go2_description: urdf/go2.urdf missing\n"
    "  Expected: ${_go2_urdf}\n"
    "========================================================\n"
  )
endif()

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING PATTERN "*.urdf" PATTERN "*.xacro"
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
  FILES_MATCHING PATTERN "*.dae" PATTERN "*.stl"
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
