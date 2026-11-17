cmake_minimum_required(VERSION 3.8)
project(fb70_description)

find_package(ament_cmake REQUIRED)

# mjcf/ 由 graphen_mujoco/tools/export_mjcf.py 產生，meshdir 為 FullBodyT70 的絕對路徑（不複製 mesh）；
# config/ 為依關節的 PD 增益（graphen_bringup 在 robot:=fb70 時載入）
install(DIRECTORY mjcf config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
