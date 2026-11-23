cmake_minimum_required(VERSION 3.8)
project(graphen_arm_ik)

# 核心函式庫不依賴 rclcpp（planner plugin 與 choreographer 也會使用）；
# 只有 arm_ik_server 執行檔連結 rclcpp

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Eigen 在未最佳化的建置下非常慢，沒有指定時預設為 Release
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(graphen_interfaces REQUIRED)
find_package(graphen_locomotion REQUIRED)
find_package(pluginlib REQUIRED)
# MuJoCo 的尋找邏輯由 graphen_mujoco 提供（cmake/graphen_mujoco-extras.cmake）
find_package(graphen_mujoco REQUIRED)
graphen_find_mujoco()
if(NOT MUJOCO_FOUND)
  ament_package()
  return()
endif()

add_library(${PROJECT_NAME} SHARED
  src/arm_config.cpp
  src/arm_ik.cpp
  src/arm_model.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
# header 會 include mujoco.h 與 Eigen，所以兩者都是 PUBLIC
target_link_libraries(${PROJECT_NAME}
  PUBLIC Eigen3::Eigen mujoco::mujoco
  PRIVATE yaml-cpp ament_index_cpp::ament_index_cpp)
# libmujoco.so 安裝在同一個 lib 目錄
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "$ORIGIN")

# 離線自我檢查：載入 arm_ik.yaml、列出允許的接觸、以隨機目標驗證 IK / swivel / 伺服
add_executable(arm_ik_check tools/arm_ik_check.cpp)
target_link_libraries(arm_ik_check PRIVATE ${PROJECT_NAME})
set_target_properties(arm_ik_check PROPERTIES INSTALL_RPATH "$ORIGIN/..")

# IK service（~/solve、~/forward）：兩套 UI 共用
add_executable(arm_ik_server src/arm_ik_server.cpp)
target_link_libraries(arm_ik_server PRIVATE ${PROJECT_NAME}
  rclcpp::rclcpp ${sensor_msgs_TARGETS} ${geometry_msgs_TARGETS} ${graphen_interfaces_TARGETS})
set_target_properties(arm_ik_server PROPERTIES INSTALL_RPATH "$ORIGIN/..")

# 視覺伺服的 planner plugin（graphen_arm_ik/ArmServoAlgorithm）；不依賴 rclcpp
add_library(${PROJECT_NAME}_plugins SHARED src/arm_servo_algorithm.cpp)
target_link_libraries(${PROJECT_NAME}_plugins PRIVATE
  ${PROJECT_NAME}
  graphen_locomotion::graphen_locomotion
  pluginlib::pluginlib
)
set_target_properties(${PROJECT_NAME}_plugins PROPERTIES INSTALL_RPATH "$ORIGIN")
# base class 定義在 graphen_locomotion，plugin 描述檔掛在它的名稱下
pluginlib_export_plugin_description_file(graphen_locomotion graphen_arm_ik_plugins.xml)

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
# 裝到 lib/<pkg>，可以用 ros2 run graphen_arm_ik arm_ik_check 執行
install(
  TARGETS ${PROJECT_NAME}_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS arm_ik_check arm_ik_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
graphen_install_mujoco_runtime()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 graphen_mujoco)
# 匯出的 target 連結 mujoco::mujoco：使用端 find_package 時建立它
ament_package(CONFIG_EXTRAS cmake/graphen_arm_ik-extras.cmake)
