cmake_minimum_required(VERSION 3.8)
project(graphen_mujoco)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# MuJoCo C 函式庫（3.11.0，環境依賴）的尋找邏輯與 graphen_arm_ik 共用，見 cmake/graphen_mujoco-extras.cmake
include(cmake/graphen_mujoco-extras.cmake)
graphen_find_mujoco()
if(NOT MUJOCO_FOUND)
  ament_package(CONFIG_EXTRAS cmake/graphen_mujoco-extras.cmake)
  return()
endif()

add_library(graphen_mujoco_hardware SHARED
  src/mujoco_system.cpp
)
target_include_directories(graphen_mujoco_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(graphen_mujoco_hardware mujoco::mujoco)
ament_target_dependencies(graphen_mujoco_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
# libmujoco.so 與 plugin 安裝在同一個 lib 目錄
set_target_properties(graphen_mujoco_hardware PROPERTIES INSTALL_RPATH "$ORIGIN")

pluginlib_export_plugin_description_file(hardware_interface graphen_mujoco.xml)

install(TARGETS graphen_mujoco_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# 以 SONAME 安裝到 plugin 旁邊，執行時由 RPATH 找到，不需要 LD_LIBRARY_PATH
graphen_install_mujoco_runtime()
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY tools
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS cmake/graphen_mujoco-extras.cmake)
