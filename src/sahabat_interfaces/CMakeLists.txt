cmake_minimum_required(VERSION 3.8)
project(sahabat_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MapInfo.msg"
  "msg/OperatorStatus.msg"
  "msg/RouteSegment.msg"
  "msg/TeleopCommand.msg"
  "msg/Waypoint.msg"
  "msg/WaypointSetInfo.msg"
  "srv/ControlLease.srv"
  "srv/GetWaypoints.srv"
  "srv/GetWaypointGraph.srv"
  "srv/ListMaps.srv"
  "srv/ListWaypointSets.srv"
  "srv/LocalizationRecovery.srv"
  "srv/LoadMap.srv"
  "srv/ManageWaypointSet.srv"
  "srv/PatrolCommand.srv"
  "srv/SaveMap.srv"
  "srv/SaveDock.srv"
  "srv/SaveWaypointGraph.srv"
  "srv/SaveWaypoints.srv"
  "srv/SetEmergencyStop.srv"
  "srv/SetMode.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
