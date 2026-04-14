cmake_minimum_required(VERSION 3.8)
project(vendor_check)

find_package(ament_cmake REQUIRED)

set(VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

if(NOT EXISTS "${VENDOR_DIR}/unitree_go/package.xml")
  message(FATAL_ERROR
    "\n"
    "========================================================\n"
    "  MISSING VENDOR DEPENDENCY: unitree_go\n"
    "\n"
    "  Create a symlink to your unitree_ros2 installation:\n"
    "\n"
    "    ln -s ~/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go \\\n"
    "          ${VENDOR_DIR}/unitree_go\n"
    "\n"
    "  See src/vendor/README.md for details.\n"
    "========================================================\n"
  )
endif()

if(NOT EXISTS "${VENDOR_DIR}/unitree_api/package.xml")
  message(FATAL_ERROR
    "\n"
    "========================================================\n"
    "  MISSING VENDOR DEPENDENCY: unitree_api\n"
    "\n"
    "  Create a symlink to your unitree_ros2 installation:\n"
    "\n"
    "    ln -s ~/unitree_ros2/cyclonedds_ws/src/unitree/unitree_api \\\n"
    "          ${VENDOR_DIR}/unitree_api\n"
    "\n"
    "  See src/vendor/README.md for details.\n"
    "========================================================\n"
  )
endif()

message(STATUS "Vendor check passed: unitree_go and unitree_api found")

ament_package()
