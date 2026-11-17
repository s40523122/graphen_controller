cmake_minimum_required(VERSION 3.5)

project(ti5_170a_graphen)

find_package(ament_cmake REQUIRED)

# 安裝資源資料夾
install(DIRECTORY launch urdf meshes config mjcf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()