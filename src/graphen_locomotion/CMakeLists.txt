cmake_minimum_required(VERSION 3.8)
project(graphen_locomotion)

# 介面只有 header，不依賴 rclcpp / ros2_control：外部演算法只需要依賴這個套件。
# 內建 plugin（範例演算法、估測器）另外編成 graphen_locomotion_plugins，只有它使用 pluginlib、Eigen、urdfdom

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)

# 介面（header-only）
add_library(${PROJECT_NAME} INTERFACE)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# 內建 plugin
add_library(${PROJECT_NAME}_plugins SHARED
  src/imu_leaky_estimator.cpp
  src/sine_wave_algorithm.cpp
)
target_link_libraries(${PROJECT_NAME}_plugins PRIVATE
  ${PROJECT_NAME} pluginlib::pluginlib Eigen3::Eigen urdfdom::urdfdom_model)

# base class 也定義在這個套件中，plugin 描述檔掛在本套件名稱下
pluginlib_export_plugin_description_file(${PROJECT_NAME} graphen_locomotion_plugins.xml)

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
)
install(
  TARGETS ${PROJECT_NAME}_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# 外部演算法以 PLUGINLIB_EXPORT_CLASS 導出 plugin，一併匯出 pluginlib 方便使用
ament_export_dependencies(pluginlib)
ament_package()
