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

# MuJoCo C 函式庫：環境依賴，各機器依自己的架構安裝（x86_64 / aarch64 的 release tarball）。
# 版本鎖定為 3.11.0：MJCF 由訓練場景以同一版匯出，其他版本的物理結果可能不同
set(MUJOCO_VERSION 3.11.0)
set(MUJOCO_VERSION_HEADER 3011000)  # mujoco.h 的 mjVERSION_HEADER
if(DEFINED ENV{MUJOCO_DIR})
  set(_mujoco_default_dir $ENV{MUJOCO_DIR})
else()
  set(_mujoco_default_dir "")
endif()
set(MUJOCO_DIR "${_mujoco_default_dir}" CACHE PATH
  "MuJoCo ${MUJOCO_VERSION} install directory (include/mujoco/mujoco.h, lib/libmujoco.so)")
# MUJOCO_DIR 沒有指定時依序找這些位置
set(_mujoco_search_dirs
  ${MUJOCO_DIR}
  $ENV{HOME}/.local/mujoco-${MUJOCO_VERSION}
  /opt/mujoco-${MUJOCO_VERSION}
)

# 從原始碼建置安裝的 MuJoCo 附有 CMake 設定檔；release tarball 沒有，改用 find_path / find_library
find_package(mujoco ${MUJOCO_VERSION} EXACT CONFIG QUIET HINTS ${_mujoco_search_dirs})
if(NOT TARGET mujoco::mujoco)
  find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
    HINTS ${_mujoco_search_dirs} PATH_SUFFIXES include NO_DEFAULT_PATH)
  find_library(MUJOCO_LIBRARY NAMES mujoco
    HINTS ${_mujoco_search_dirs} PATH_SUFFIXES lib NO_DEFAULT_PATH)
  if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
    string(REPLACE ";" "\n    " _mujoco_searched "${_mujoco_search_dirs}")
    message(WARNING
      "MuJoCo ${MUJOCO_VERSION} not found — graphen_mujoco will NOT be built.\n"
      "  Searched:\n    ${_mujoco_searched}\n"
      "  Download mujoco-${MUJOCO_VERSION}-linux-${CMAKE_SYSTEM_PROCESSOR}.tar.gz from\n"
      "  https://github.com/google-deepmind/mujoco/releases and extract to\n"
      "  ~/.local/mujoco-${MUJOCO_VERSION} (or set MUJOCO_DIR).")
    ament_package()
    return()
  endif()
  add_library(mujoco::mujoco SHARED IMPORTED)
  set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION ${MUJOCO_LIBRARY}
    INTERFACE_INCLUDE_DIRECTORIES ${MUJOCO_INCLUDE_DIR}
  )
endif()

# 版本檢查以 header 為準（find_package 的版本檢查只涵蓋有設定檔的安裝）
get_target_property(_mujoco_include_dirs mujoco::mujoco INTERFACE_INCLUDE_DIRECTORIES)
find_file(_mujoco_header mujoco/mujoco.h PATHS ${_mujoco_include_dirs} NO_DEFAULT_PATH)
file(STRINGS ${_mujoco_header} _mujoco_version_line REGEX "^#define mjVERSION_HEADER ")
string(REGEX REPLACE "^#define mjVERSION_HEADER ([0-9]+).*" "\\1"
  _mujoco_found_version "${_mujoco_version_line}")
if(NOT _mujoco_found_version STREQUAL MUJOCO_VERSION_HEADER)
  message(FATAL_ERROR "MuJoCo at ${_mujoco_header} has mjVERSION_HEADER "
    "${_mujoco_found_version}, expected ${MUJOCO_VERSION_HEADER} (${MUJOCO_VERSION})")
endif()
get_target_property(_mujoco_location mujoco::mujoco LOCATION)
file(REAL_PATH ${_mujoco_location} MUJOCO_REAL_LIBRARY)
message(STATUS "Using MuJoCo ${MUJOCO_VERSION}: ${MUJOCO_REAL_LIBRARY}")

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
install(FILES ${MUJOCO_REAL_LIBRARY} DESTINATION lib RENAME libmujoco.so.${MUJOCO_VERSION})
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

ament_package()
