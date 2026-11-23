# graphen_arm_ik 匯出的 target 連結 mujoco::mujoco；使用端 find_package(graphen_arm_ik) 時建立它
find_package(graphen_mujoco REQUIRED)
graphen_find_mujoco()
if(NOT MUJOCO_FOUND)
  message(FATAL_ERROR "graphen_arm_ik requires MuJoCo ${GRAPHEN_MUJOCO_VERSION}")
endif()
