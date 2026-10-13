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

# MuJoCo C 函式庫（取自 mujoco 3.11.0 Python wheel 內附的 libmujoco.so 與 headers，aarch64）
set(MUJOCO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco)
add_library(mujoco::mujoco SHARED IMPORTED)
set_target_properties(mujoco::mujoco PROPERTIES
  IMPORTED_LOCATION ${MUJOCO_DIR}/lib/libmujoco.so.3.11.0
  IMPORTED_SONAME libmujoco.so.3.11.0
  INTERFACE_INCLUDE_DIRECTORIES ${MUJOCO_DIR}/include
)

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
install(FILES
  ${MUJOCO_DIR}/lib/libmujoco.so.3.11.0
  DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY models tools
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
