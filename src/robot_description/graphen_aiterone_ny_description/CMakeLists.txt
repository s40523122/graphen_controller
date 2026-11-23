cmake_minimum_required(VERSION 3.8)
project(graphen_aiterone_ny_description)

find_package(ament_cmake REQUIRED)

# mjcf/ 與 meshes/hulls/ 由 scripts/ 產生後一併提交；scripts/ 為開發工具，不安裝
install(DIRECTORY urdf meshes launch rviz config mjcf DESTINATION share/${PROJECT_NAME})

ament_package()
