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

# 訓練輸出的部署資料夾：t170_policy_config.h（obs 組成、q_home、action scale）與 ONNX Runtime。
# 重新訓練後更新該資料夾（同時更新 YAML 的 model_path）再重新建置即可
set(T170_POLICY_DIR "/devtest/FullBodyT70/cpp_interface" CACHE PATH
  "FullBodyT70 cpp_interface directory (t170_policy_config.h and third_party/onnxruntime)")
set(ORT_DIR ${T170_POLICY_DIR}/third_party/onnxruntime)
if(NOT EXISTS ${T170_POLICY_DIR}/src/t170_policy_config.h OR
   NOT EXISTS ${ORT_DIR}/lib/libonnxruntime.so.1)
  message(FATAL_ERROR "T170_POLICY_DIR=${T170_POLICY_DIR} must contain src/t170_policy_config.h "
    "and third_party/onnxruntime/lib/libonnxruntime.so.1")
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
target_include_directories(graphen_rl_policy_plugins PRIVATE ${T170_POLICY_DIR}/src)
target_link_libraries(graphen_rl_policy_plugins
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
