cmake_minimum_required(VERSION 3.8)
project(graphen_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch motions
  DESTINATION share/${PROJECT_NAME}
)

# IMU 方向性驗證工具：ros2 run graphen_bringup imu_orientation_check
install(
  PROGRAMS scripts/imu_orientation_check.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_orientation_check
)

# 舞曲庫工具（發佈 / 列出 / 讀回）：ros2 run graphen_bringup choreo_publish
install(
  PROGRAMS scripts/choreo_publish.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME choreo_publish
)

# 播放介面的情境回歸：ros2 run graphen_bringup playback_scenarios（對著已啟動的 mock / MuJoCo bringup）
install(
  PROGRAMS scripts/playback_scenarios.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME playback_scenarios
)

# 把文字檔發布成 latched topic（bringup 用來發布 SRDF 到 /robot_description_semantic）
install(
  PROGRAMS scripts/file_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME file_publisher
)

ament_package()
