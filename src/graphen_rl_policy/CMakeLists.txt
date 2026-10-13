cmake_minimum_required(VERSION 3.8)
project(graphen_rl_policy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(graphen_locomotion REQUIRED)
find_package(pluginlib REQUIRED)

find_package(ament_index_cpp REQUIRED)

# policy/：policy.onnx 與對應的 t170_policy_config.h（obs 組成、q_home、action scale），兩者必須來自同一次訓練。
# 換 policy 時兩個檔案一起換掉再重新建置（來源：FullBodyT70/cpp_interface）。
# third_party/onnxruntime：ONNX Runtime 1.31.0（aarch64，不進版控，從 FullBodyT70/cpp_interface 複製）
set(T170_POLICY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/policy CACHE PATH
  "Directory with policy.onnx and t170_policy_config.h")
set(ORT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime CACHE PATH
  "ONNX Runtime directory (include/ and lib/libonnxruntime.so.1)")
if(NOT EXISTS ${T170_POLICY_DIR}/t170_policy_config.h OR NOT EXISTS ${T170_POLICY_DIR}/policy.onnx)
  message(FATAL_ERROR "T170_POLICY_DIR=${T170_POLICY_DIR} must contain policy.onnx and t170_policy_config.h")
endif()
if(NOT EXISTS ${ORT_DIR}/lib/libonnxruntime.so.1)
  message(FATAL_ERROR "ORT_DIR=${ORT_DIR} must contain lib/libonnxruntime.so.1 "
    "(copy FullBodyT70/cpp_interface/third_party/onnxruntime)")
endif()

add_library(onnxruntime::onnxruntime SHARED IMPORTED)
file(REAL_PATH ${ORT_DIR}/lib/libonnxruntime.so.1 ORT_LIBRARY)
set_target_properties(onnxruntime::onnxruntime PROPERTIES
  IMPORTED_LOCATION ${ORT_LIBRARY}
  IMPORTED_SONAME libonnxruntime.so.1
  INTERFACE_INCLUDE_DIRECTORIES ${ORT_DIR}/include
)

add_library(graphen_rl_policy_plugins SHARED
  src/t170_velocity_policy.cpp
)
target_include_directories(graphen_rl_policy_plugins PRIVATE ${T170_POLICY_DIR})
target_link_libraries(graphen_rl_policy_plugins
  ament_index_cpp::ament_index_cpp
  graphen_locomotion::graphen_locomotion
  pluginlib::pluginlib
  onnxruntime::onnxruntime
)
# libonnxruntime 與 plugin 安裝在同一個 lib 目錄
set_target_properties(graphen_rl_policy_plugins PROPERTIES INSTALL_RPATH "$ORIGIN")

pluginlib_export_plugin_description_file(graphen_locomotion graphen_rl_policy_plugins.xml)

install(TARGETS graphen_rl_policy_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# 以 SONAME 安裝，plugin 載入時由 RPATH 找到
install(FILES ${ORT_LIBRARY} DESTINATION lib RENAME libonnxruntime.so.1)
# YAML 以 package://graphen_rl_policy/policy/policy.onnx 指定
install(FILES ${T170_POLICY_DIR}/policy.onnx ${T170_POLICY_DIR}/t170_policy_config.h
  DESTINATION share/${PROJECT_NAME}/policy)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
