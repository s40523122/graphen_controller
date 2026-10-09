cmake_minimum_required(VERSION 3.8)
project(ti5_hardware)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(can_driver REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml-cpp REQUIRED)

# hardware_interface plugin (Ti5Hardware)
add_library(ti5_control_plugin SHARED
  src/ti5_hardware.cpp
  src/joint_safety_control.cpp
)
target_include_directories(ti5_control_plugin PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ti5_control_plugin
  can_driver::can_driver_lib
  yaml-cpp
)
ament_target_dependencies(ti5_control_plugin
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  urdf
)
install(TARGETS ti5_control_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES ti5_control_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# 讓 pluginlib/ament_index 能找到此 hardware_interface plugin
pluginlib_export_plugin_description_file(hardware_interface ti5_control_plugin.xml)

ament_export_dependencies(hardware_interface pluginlib can_driver rclcpp_lifecycle urdf)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
