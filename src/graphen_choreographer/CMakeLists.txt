cmake_minimum_required(VERSION 3.8)
project(graphen_choreographer)

# 編舞（多軌 keyframe 的組合動作）。程式碼不依賴 rclcpp / ros2_control：
#   graphen_choreographer          腳本的解析、取樣與限位檢查（共用函式庫）
#   graphen_choreographer_plugins  graphen_locomotion::LocomotionAlgorithm plugin（ChoreographerAlgorithm）
#   choreo_check            離線檢查工具

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(graphen_locomotion REQUIRED)
find_package(pluginlib REQUIRED)
find_package(urdfdom REQUIRED)
find_package(yaml-cpp REQUIRED)

# yaml-cpp、urdfdom 只在實作中使用，header 不需要
add_library(${PROJECT_NAME} SHARED
  src/choreography.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp urdfdom::urdfdom_model)

add_library(${PROJECT_NAME}_plugins SHARED
  src/choreographer_algorithm.cpp
)
target_link_libraries(${PROJECT_NAME}_plugins PRIVATE
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  graphen_locomotion::graphen_locomotion
  pluginlib::pluginlib
)

# 離線檢查用：列出各 joint 的範圍、峰值速度、限位違規與建議的 max_steps，並可輸出 CSV
add_executable(choreo_check tools/choreo_check.cpp)
target_link_libraries(choreo_check PRIVATE ${PROJECT_NAME})

# base class 定義在 graphen_locomotion，plugin 描述檔掛在它的名稱下
pluginlib_export_plugin_description_file(graphen_locomotion graphen_choreographer_plugins.xml)

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
install(
  TARGETS ${PROJECT_NAME}_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# 裝到 lib/<pkg>，可以用 ros2 run graphen_choreographer choreo_check 執行
install(
  TARGETS choreo_check
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
