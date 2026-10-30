cmake_minimum_required(VERSION 3.8)
project(graphen_rl_policy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(graphen_locomotion REQUIRED)
find_package(pluginlib REQUIRED)

# third_party/onnxruntime：ONNX Runtime 1.31.0（include/ 與 lib/libonnxruntime.so.1），不進版控。
# 依建置機器的架構準備：Orin 用 aarch64 版（FullBodyT70/cpp_interface/third_party/onnxruntime），
# x86_64 開發機用 github.com/microsoft/onnxruntime releases 的 onnxruntime-linux-x64-1.31.0.tgz
set(ORT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime CACHE PATH
  "ONNX Runtime directory (include/ and lib/libonnxruntime.so.1)")
if(NOT EXISTS ${ORT_DIR}/lib/libonnxruntime.so.1)
  message(FATAL_ERROR "ORT_DIR=${ORT_DIR} must contain lib/libonnxruntime.so.1 "
    "built for ${CMAKE_SYSTEM_PROCESSOR}")
endif()

add_library(onnxruntime::onnxruntime SHARED IMPORTED)
file(REAL_PATH ${ORT_DIR}/lib/libonnxruntime.so.1 ORT_LIBRARY)
set_target_properties(onnxruntime::onnxruntime PROPERTIES
  IMPORTED_LOCATION ${ORT_LIBRARY}
  IMPORTED_SONAME libonnxruntime.so.1
  INTERFACE_INCLUDE_DIRECTORIES ${ORT_DIR}/include
)

# 共用函式庫：OnnxModel、PolicyRunner、ObsHistory。其他套件也可以繼承 PolicyRunner 寫 policy plugin；
# ORT 只在實作中使用，header 不需要 ORT
add_library(${PROJECT_NAME} SHARED
  src/onnx_model.cpp
  src/policy_runner.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC graphen_locomotion::graphen_locomotion
  PRIVATE ament_index_cpp::ament_index_cpp onnxruntime::onnxruntime
)

# policy plugin：一個 policy 家族（obs / action 組法）一個 class
add_library(${PROJECT_NAME}_plugins SHARED
  src/t170_velocity_policy.cpp
  src/ti5_flat_policy.cpp
)
target_link_libraries(${PROJECT_NAME}_plugins
  ${PROJECT_NAME}
  pluginlib::pluginlib
)

# libonnxruntime 與兩個函式庫安裝在同一個 lib 目錄，載入時由 RPATH 找到
set_target_properties(${PROJECT_NAME} ${PROJECT_NAME}_plugins PROPERTIES INSTALL_RPATH "$ORIGIN")

pluginlib_export_plugin_description_file(graphen_locomotion graphen_rl_policy_plugins.xml)

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
install(TARGETS ${PROJECT_NAME}_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# 以 SONAME 安裝
install(FILES ${ORT_LIBRARY} DESTINATION lib RENAME libonnxruntime.so.1)
# onnx 模型；YAML 以 package://graphen_rl_policy/policy/<檔名>.onnx 指定，並以 model_sha256 綁定
install(DIRECTORY policy/ DESTINATION share/${PROJECT_NAME}/policy)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(graphen_locomotion)
ament_package()
