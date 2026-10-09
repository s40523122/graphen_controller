cmake_minimum_required(VERSION 3.8)
project(can_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)

add_library(can_driver_lib SHARED
  src/can_config_loader.cpp
  src/socketcan_interface.cpp
  src/motor_control.cpp
  src/can_driver_builder.cpp
  src/can_manager.cpp
)

target_include_directories(can_driver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(can_driver_lib
  rclcpp
)

add_executable(can_driver_test_node
  src/ti5_can_test_node.cpp
)

target_link_libraries(can_driver_test_node
  can_driver_lib
)

ament_target_dependencies(can_driver_test_node
  rclcpp
  ament_index_cpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  TARGETS can_driver_lib
  EXPORT can_driver_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS can_driver_test_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(EXPORT can_driver_targets
  FILE can_driver_targets.cmake
  NAMESPACE can_driver::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include)
ament_export_targets(can_driver_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

ament_package()

