# MuJoCo C 函式庫的尋找與安裝，graphen_mujoco 與其他使用 MuJoCo 的套件（graphen_arm_ik）共用。
# graphen_mujoco 直接 include；其他套件 find_package(graphen_mujoco) 後即可使用（ament CONFIG_EXTRAS）。
#
# MuJoCo 是環境依賴，各機器依自己的架構安裝（x86_64 / aarch64 的 release tarball）。
# 版本鎖定為 3.11.0：MJCF 由訓練場景以同一版匯出，其他版本的物理結果可能不同。
#
#   graphen_find_mujoco()
#     找到時建立 target mujoco::mujoco，並設定 MUJOCO_FOUND=TRUE、MUJOCO_REAL_LIBRARY（實際的 .so）；
#     找不到時 MUJOCO_FOUND=FALSE 並印出警告（由呼叫端決定是否略過建置）。版本不符時 FATAL_ERROR
#   graphen_install_mujoco_runtime()
#     把 libmujoco.so 以 SONAME 安裝到套件的 lib 目錄；搭配 INSTALL_RPATH "$ORIGIN"，
#     執行時不需要 LD_LIBRARY_PATH

set(GRAPHEN_MUJOCO_VERSION 3.11.0)
set(GRAPHEN_MUJOCO_VERSION_HEADER 3011000)  # mujoco.h 的 mjVERSION_HEADER

macro(graphen_find_mujoco)
  if(DEFINED ENV{MUJOCO_DIR})
    set(_graphen_mujoco_default_dir $ENV{MUJOCO_DIR})
  else()
    set(_graphen_mujoco_default_dir "")
  endif()
  set(MUJOCO_DIR "${_graphen_mujoco_default_dir}" CACHE PATH
    "MuJoCo ${GRAPHEN_MUJOCO_VERSION} install directory (include/mujoco/mujoco.h, lib/libmujoco.so)")
  # MUJOCO_DIR 沒有指定時依序找這些位置
  set(_graphen_mujoco_search_dirs
    ${MUJOCO_DIR}
    $ENV{HOME}/.local/mujoco-${GRAPHEN_MUJOCO_VERSION}
    /opt/mujoco-${GRAPHEN_MUJOCO_VERSION}
  )

  set(MUJOCO_FOUND TRUE)
  # 從原始碼建置安裝的 MuJoCo 附有 CMake 設定檔；release tarball 沒有，改用 find_path / find_library
  if(NOT TARGET mujoco::mujoco)
    find_package(mujoco ${GRAPHEN_MUJOCO_VERSION} EXACT CONFIG QUIET HINTS ${_graphen_mujoco_search_dirs})
  endif()
  if(NOT TARGET mujoco::mujoco)
    find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
      HINTS ${_graphen_mujoco_search_dirs} PATH_SUFFIXES include NO_DEFAULT_PATH)
    find_library(MUJOCO_LIBRARY NAMES mujoco
      HINTS ${_graphen_mujoco_search_dirs} PATH_SUFFIXES lib NO_DEFAULT_PATH)
    if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
      string(REPLACE ";" "\n    " _graphen_mujoco_searched "${_graphen_mujoco_search_dirs}")
      message(WARNING
        "MuJoCo ${GRAPHEN_MUJOCO_VERSION} not found — ${PROJECT_NAME} will NOT be built.\n"
        "  Searched:\n    ${_graphen_mujoco_searched}\n"
        "  Download mujoco-${GRAPHEN_MUJOCO_VERSION}-linux-${CMAKE_SYSTEM_PROCESSOR}.tar.gz from\n"
        "  https://github.com/google-deepmind/mujoco/releases and extract to\n"
        "  ~/.local/mujoco-${GRAPHEN_MUJOCO_VERSION} (or set MUJOCO_DIR).")
      set(MUJOCO_FOUND FALSE)
    else()
      add_library(mujoco::mujoco SHARED IMPORTED)
      set_target_properties(mujoco::mujoco PROPERTIES
        IMPORTED_LOCATION ${MUJOCO_LIBRARY}
        INTERFACE_INCLUDE_DIRECTORIES ${MUJOCO_INCLUDE_DIR}
      )
    endif()
  endif()

  if(MUJOCO_FOUND)
    # 版本檢查以 header 為準（find_package 的版本檢查只涵蓋有設定檔的安裝）
    get_target_property(_graphen_mujoco_include_dirs mujoco::mujoco INTERFACE_INCLUDE_DIRECTORIES)
    find_file(_graphen_mujoco_header mujoco/mujoco.h
      PATHS ${_graphen_mujoco_include_dirs} NO_DEFAULT_PATH)
    file(STRINGS ${_graphen_mujoco_header} _graphen_mujoco_version_line
      REGEX "^#define mjVERSION_HEADER ")
    string(REGEX REPLACE "^#define mjVERSION_HEADER ([0-9]+).*" "\\1"
      _graphen_mujoco_found_version "${_graphen_mujoco_version_line}")
    if(NOT _graphen_mujoco_found_version STREQUAL GRAPHEN_MUJOCO_VERSION_HEADER)
      message(FATAL_ERROR "MuJoCo at ${_graphen_mujoco_header} has mjVERSION_HEADER "
        "${_graphen_mujoco_found_version}, expected ${GRAPHEN_MUJOCO_VERSION_HEADER} "
        "(${GRAPHEN_MUJOCO_VERSION})")
    endif()
    get_target_property(_graphen_mujoco_location mujoco::mujoco LOCATION)
    file(REAL_PATH ${_graphen_mujoco_location} MUJOCO_REAL_LIBRARY)
    message(STATUS "Using MuJoCo ${GRAPHEN_MUJOCO_VERSION}: ${MUJOCO_REAL_LIBRARY}")
  endif()
endmacro()

macro(graphen_install_mujoco_runtime)
  install(FILES ${MUJOCO_REAL_LIBRARY} DESTINATION lib
    RENAME libmujoco.so.${GRAPHEN_MUJOCO_VERSION})
endmacro()
