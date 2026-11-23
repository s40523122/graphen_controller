cmake_minimum_required(VERSION 3.8)
project(graphen_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 手臂 IK（graphen_arm_ik）：SolveArmIk / ArmForward（arm_ik_server）、ArmPoseTarget（視覺伺服的目標）
# 播放類 action 共用 msg/Playback*.msg（選項、結果、feedback），各自只定義專用的 goal 欄位；
# PlaybackState 為 controller 的 ~/playback_state（目前在播什麼）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlaybackOptions.msg"
  "msg/PlaybackResult.msg"
  "msg/PlaybackFeedback.msg"
  "msg/PlaybackState.msg"
  "msg/ArmPoseTarget.msg"
  "action/SwitchMode.action"
  "action/PlayChoreo.action"
  "action/PlayBvh.action"
  "action/RunPolicy.action"
  "srv/PublishChoreo.srv"
  "srv/ListChoreos.srv"
  "srv/GetChoreo.srv"
  "srv/Preload.srv"
  "srv/SolveArmIk.srv"
  "srv/ArmForward.srv"
  "srv/TcpJog.srv"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
