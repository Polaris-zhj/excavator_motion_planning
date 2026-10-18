cmake_minimum_required(VERSION 3.8)
project(excavator_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CollisionObject.msg"
  "msg/PlanningScene.msg"
  "msg/TrajectoryUpdate.msg"
  "msg/ExecutionStatus.msg"
  "action/PlanTrajectory.action"
  "action/ExecuteTrajectory.action"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs sensor_msgs
    shape_msgs std_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
