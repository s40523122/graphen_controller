cmake_minimum_required(VERSION 3.8)
project(graphen_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_DEPENDS
  behaviortree_cpp_v3
  rclcpp
  std_msgs
)

find_package(ament_cmake REQUIRED)
foreach(dep IN ITEMS ${THIS_PACKAGE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

# 節點庫 graphen_bt_nodes 於執行期以 plugin 載入，不需在編譯期連結
add_executable(bt_runner src/bt_runner.cpp)
target_compile_features(bt_runner PRIVATE cxx_std_17)
ament_target_dependencies(bt_runner ${THIS_PACKAGE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS bt_runner
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
