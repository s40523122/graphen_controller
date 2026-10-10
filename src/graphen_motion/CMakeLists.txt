cmake_minimum_required(VERSION 3.8)
project(graphen_motion)

# 程式碼不依賴 rclcpp；使用 ament_cmake 只是為了安裝與匯出 target，方便其他 ROS 套件 find_package

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Eigen 在未最佳化的建置下非常慢，沒有指定時預設為 Release
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/bvh_clip.cpp
  src/bvh_retargeter.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen PRIVATE urdfdom::urdfdom_model)

# 離線檢查用：把整段 BVH 轉成 CSV，並印出分組、殘差、速度等統計
add_executable(bvh_retarget_cli tools/bvh_retarget_cli.cpp)
target_link_libraries(bvh_retarget_cli PRIVATE ${PROJECT_NAME})

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
# 裝到 lib/<pkg>，可以用 ros2 run graphen_motion bvh_retarget_cli 執行
install(
  TARGETS bvh_retarget_cli
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
