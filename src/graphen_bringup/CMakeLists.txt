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

ament_package()
