cmake_minimum_required(VERSION 3.8)
project(graphen_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_index_cpp
  controller_interface
  geometry_msgs
  graphen_interfaces
  hardware_interface
  lifecycle_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

find_package(ament_cmake REQUIRED)
find_package(graphen_control_utils REQUIRED)
find_package(graphen_locomotion REQUIRED)
find_package(graphen_motion REQUIRED)
foreach(dep IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/bvh_motion.cpp
  src/locomotion_planner.cpp
  src/plugin_motion.cpp
  src/switch_group_controller.cpp
  src/sub_group_controller.cpp
  src/target_motion.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  graphen_control_utils::graphen_control_utils
  graphen_locomotion::graphen_locomotion
  graphen_motion::graphen_motion
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# 使用 pluginlib 導出 plugin 描述檔，讓 controller_manager 能載入
pluginlib_export_plugin_description_file(controller_interface graphen_controllers.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(graphen_control_utils graphen_locomotion graphen_motion ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
