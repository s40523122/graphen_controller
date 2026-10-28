cmake_minimum_required(VERSION 3.8)
project(t170a_description)

find_package(ament_cmake REQUIRED)

# mjcf/ 與 urdf/ 以相對路徑 ../meshes/ 引用 mesh，安裝後的目錄結構必須維持一致。
# source/ 為 CAD 匯出的原始檔（CSV、原始 URDF），只作為來源紀錄，不安裝
install(DIRECTORY mjcf urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
