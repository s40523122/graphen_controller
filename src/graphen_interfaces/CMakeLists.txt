cmake_minimum_required(VERSION 3.8)
project(graphen_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 播放類 action 共用 msg/Playback*.msg（選項、結果、feedback），各自只定義專用的 goal 欄位
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlaybackOptions.msg"
  "msg/PlaybackResult.msg"
  "msg/PlaybackFeedback.msg"
  "action/SwitchMode.action"
  "action/PlayChoreo.action"
  "action/PlayBvh.action"
  "action/RunPolicy.action"
  "srv/PublishChoreo.srv"
  "srv/ListChoreos.srv"
  "srv/GetChoreo.srv"
  "srv/Preload.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
