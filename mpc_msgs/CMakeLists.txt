cmake_minimum_required(VERSION 3.8)
project(mpc_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DTLane.msg"
  "msg/Waypoint.msg"
  "msg/WaypointState.msg"
  "msg/Lane.msg"
  "msg/ControlCommand.msg"
  "msg/ControlCommandStamped.msg"
  "msg/Gear.msg"
  "msg/Trajectory.msg"
  "msg/TrajectoryPoint.msg"
  "msg/SteeringReport.msg"
  "msg/AckermannLateralCommand.msg"
  "msg/Path.msg"
  "msg/PathPoint.msg"
  "msg/Float32MultiArrayStamped.msg"
  "msg/MultiArrayLayout.msg"
  "msg/MultiArrayDimension.msg"
  "msg/VehicleStatus.msg"
  "msg/PathPointWithLaneId.msg"
  "msg/PathWithLaneId.msg"
  "msg/VehicleOdometry.msg"
  "msg/OperationModeState.msg"
  "msg/VehicleState.msg"
  "msg/Direction.msg"
  "msg/HybridLane.msg"
  "msg/HybridTrajectory.msg"
  DEPENDENCIES builtin_interfaces nav_msgs geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
