cmake_minimum_required(VERSION 3.8)
project(dexterous_hand_hardware)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(dexterous_hand_hardware SHARED
  src/dexterous_hand_hardware.cpp
  src/rs485_comm.cpp
)

target_include_directories(dexterous_hand_hardware PUBLIC
  include
)

ament_target_dependencies(dexterous_hand_hardware
  rclcpp
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(
  hardware_interface dexterous_hand_hardware.xml)

install(TARGETS dexterous_hand_hardware
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_libraries(dexterous_hand_hardware)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
)

ament_package()
