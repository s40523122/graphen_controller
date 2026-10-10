cmake_minimum_required(VERSION 3.8)
project(graphen_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch motions
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
