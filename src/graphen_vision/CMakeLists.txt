cmake_minimum_required(VERSION 3.8)
project(graphen_vision)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 偵測與鎖定的共用邏輯（不依賴 rclpy），節點與離線工具共用
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)

# 人臉跟隨的視覺端：ros2 run graphen_vision face_tracker
install(
  PROGRAMS scripts/face_tracker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME face_tracker
)

ament_package()
