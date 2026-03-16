cmake_minimum_required(VERSION 3.8)
project(covapsy_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/DriveCommand.msg"
  "msg/TrackQuality.msg"
  "msg/DirectionState.msg"
  "msg/RecoveryState.msg"
  "msg/RaceTelemetry.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
