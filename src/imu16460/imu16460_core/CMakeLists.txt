cmake_minimum_required(VERSION 3.8)
project(imu16460_core VERSION 0.1.0 LANGUAGES CXX)

# 純 CMake 套件，不依賴 ament / ROS，可以在非 ROS 環境建置

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 定義 BUILD_TESTING 並產生 colcon test 需要的 DartConfiguration
include(CTest)

find_package(Threads REQUIRED)
include(GNUInstallDirs)

add_library(${PROJECT_NAME} SHARED
  src/crc32.cpp
  src/driver.cpp
  src/frame_parser.cpp
  src/protocol.cpp
  src/serial_port.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Threads::Threads)

add_executable(imu16460_cli tools/imu16460_cli.cpp)
target_link_libraries(imu16460_cli PRIVATE ${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  add_executable(${PROJECT_NAME}_test
    test/test_driver.cpp
    test/test_frame_parser.cpp
    test/test_protocol.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test PRIVATE ${PROJECT_NAME} GTest::gtest_main util)
  gtest_discover_tests(${PROJECT_NAME}_test)
endif()

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
# 純 CMake 套件不會登記到 ament index，無法用 `ros2 run`；
# 裝到 bin/，source install/setup.bash 後可直接執行 imu16460_cli
install(TARGETS imu16460_cli RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# 匯出 CMake config，讓其他套件以 find_package(imu16460_core) 使用 imu16460_core::imu16460_core
include(CMakePackageConfigHelpers)
set(CONFIG_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
install(
  EXPORT ${PROJECT_NAME}Targets
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CONFIG_INSTALL_DIR}
)
configure_package_config_file(
  cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CONFIG_INSTALL_DIR}
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  COMPATIBILITY SameMajorVersion
)
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CONFIG_INSTALL_DIR}
)
