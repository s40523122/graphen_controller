cmake_minimum_required(VERSION 3.8)
project(graphen_ros_bridge)

# 需要自己建立 rclcpp node（訂閱 ROS topic）的 locomotion plugin。
# graphen_locomotion 的介面與內建 plugin 維持不依賴 rclcpp，這類 plugin 集中在這個套件。
# 取樣等不依賴 ROS 的邏輯放在 include/graphen_ros_bridge/（header-only），之後可以重用

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(graphen_locomotion REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(${PROJECT_NAME}_plugins SHARED
  src/joint_trajectory_follower.cpp
)
target_include_directories(${PROJECT_NAME}_plugins PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(${PROJECT_NAME}_plugins
  graphen_locomotion::graphen_locomotion
  pluginlib::pluginlib
)
ament_target_dependencies(${PROJECT_NAME}_plugins rclcpp trajectory_msgs)

# base class 定義在 graphen_locomotion，plugin 描述檔掛在它的名稱下
pluginlib_export_plugin_description_file(graphen_locomotion graphen_ros_bridge_plugins.xml)

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
install(TARGETS ${PROJECT_NAME}_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(graphen_locomotion)
ament_package()
