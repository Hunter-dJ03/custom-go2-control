cmake_minimum_required(VERSION 3.8)
project(go2_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionState.msg"
  "msg/RobotState.msg"
  "msg/ControlState.msg"
  "msg/FaultStatus.msg"
  "msg/MotionCapability.msg"
  "msg/FootForce.msg"
  "srv/SetGait.srv"
  "srv/SetPosture.srv"
  "srv/SetBodyHeight.srv"
  "srv/SetSpeedLevel.srv"
  "srv/SetFootRaiseHeight.srv"
  "srv/TriggerMotion.srv"
  "srv/GetCapabilities.srv"
  "srv/SetControlMode.srv"
  "srv/EStop.srv"
  "srv/SportApiCall.srv"
  "action/ExecuteMotion.action"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
