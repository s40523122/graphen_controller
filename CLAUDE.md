# graphen_ws

ROS 2 Humble colcon 工作區，包含兩層：ros2_control 控制層（`SwitchGroupController` → chainable `SubGroupController` → Ti5 T170 本體，預設 mock），以及 BehaviorTree.CPP v3 行為層（`bt_runner` + plugin 節點）。兩層透過 `SwitchMode` action（切換模式）和 `~/states` topic（BT 取得 joint 狀態）連動。
架構、參數、設計決策見 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)；使用方式見 [README.md](README.md)。

## 套件
- `graphen_interfaces`：action 定義（`SwitchMode`），控制層與行為層共用
- `graphen_control_utils`：header-only 共用元件（`StateInterfacePublisher`）
- `graphen_motion`：BVH 解析與 BVH → URDF 1-DOF joint 轉換（`BvhRetargeter`）、`bvh_retarget_cli`；不依賴 rclcpp
- `graphen_locomotion`：外部 locomotion 演算法與機身狀態估測器的 plugin 介面（`LocomotionAlgorithm`、`BaseStateEstimator`、`RobotState`、`JointCommand`、`ParameterSource`），介面 header-only、**不依賴 rclcpp / ros2_control**；內建 plugin（`graphen_locomotion_plugins`）：範例 `graphen_locomotion/SineWaveAlgorithm`、只用 IMU 的估測器 `graphen_locomotion/ImuLeakyEstimator`
- `graphen_controllers`：兩個 controller plugin，plugin 描述檔為 `graphen_controllers.xml`；`LocomotionPlanner`、`Motion`（`TargetMotion` / `BvhMotion` / `PluginMotion`）是內部元件（不是 plugin）
- 模式由 YAML 的 `<mode>.type`（`target` / `hold` / `bvh` / `plugin`）決定行為，controller 不寫死任何模式；BVH 動作檔放在 `graphen_bringup/motions/`。`plugin` 模式從 planner 取得量測的關節角度 / 速度、IMU、估測的機身狀態（`base`）與 `~/cmd_vel` 速度指令（`command`）
- `graphen_bringup`：`config/controllers.yaml`（32 軸 → 6 個子群組）、`config/can_driver_config.json`（T170 實機 CAN 設定）、`motions/`、`launch/bringup.launch.py`
- `graphen_bt_nodes`：BT 節點 plugin，全部在 `register_nodes.cpp` 註冊
- `graphen_bt`：`bt_runner`、`trees/`、`config/bt_runner.yaml`、`launch/bt.launch.py`
- `src/imu16460/imu16460_core`：FSS-IMU16460-C 驅動，純 CMake、**程式碼不得 include ROS 標頭**；含 `imu16460_cli` 與 GoogleTest。協議見 [docs/IMU16460.md](docs/IMU16460.md)
- `src/imu16460/imu16460_hardware`：IMU 的 ros2_control `SensorInterface` plugin（`imu16460_hardware/Imu16460SensorInterface`）與 xacro macro
- `graphen_mujoco`：MuJoCo 模擬的 T170 本體 + IMU（`graphen_mujoco/MujocoSystem`），launch 參數 `use_mujoco`；`third_party/mujoco` 為 wheel 內附的 libmujoco，`tools/` 有 MJCF 匯出與主機端 viewer
- `graphen_rl_policy`：FullBodyT70 policy 的 `LocomotionAlgorithm` plugin（`rl_walk` 模式），ORT 與 `t170_policy_config.h` 取自 CMake 變數 `T170_POLICY_DIR`
- 外部套件（沿用原本命名，不套 `graphen_` 慣例）：
  - `ti5_170a_graphen`：robot_description。頂層 xacro 為 `urdf/ti5_170a_graphen.urdf.xacro`，本體 ros2_control 為 `urdf/ti5_170a_graphen.ros2_control.xacro`，joint 分組 SRDF 為 `config/ti5_170a_graphen.srdf`（目前只作參考）
  - `ti5_hardware`（`ti5_hardware/Ti5Hardware`，CAN）、`can_driver`：T170 關節實機
  - `dexterous_hands/`：`dexterous_hand_description`（ros2_control xacro macro）、`dexterous_hand_hardware`（RS485）；MoveIt 設定已移除

## 建置與執行
- `colcon build --symlink-install`；只改一個套件時加 `--packages-select <pkg>`
- 執行前先 `source install/setup.bash`（BT plugin 依賴 LD_LIBRARY_PATH 載入）
- `ros2 launch graphen_bringup bringup.launch.py`、`ros2 launch graphen_bt bt.launch.py`
- 本體以 launch 參數 `use_mock_hardware`（預設 true）切換 mock / CAN 實機；靈巧手以 `use_hands`（預設 false）啟用，mock / 實機跟隨 `use_mock_hardware`
- IMU 以 launch 參數 `use_mock_imu`（預設 true）切換 mock / 實機。mock 提供與實機同名的 `imu_sensor/*`（10 通道 + temperature），以 `sensor_forwarder`（`/sensor_forwarder/commands`，10 個值）寫入模擬值；兩者介面名稱必須保持一致。實機設定放在 `config/controllers_imu16460.yaml`

## 慣例
- 套件名稱：`graphen_<功能>`
- namespace：控制層 `graphen::control`，行為層 `graphen::bt`
- controller plugin 名稱：`graphen_control/<Class>`
- header 放在 `include/<pkg>/`，include guard 為 `<PKG>__<FILE>_HPP_`
- controller 參數：在 `on_init` 用 `auto_declare` 宣告，在 `on_configure` 讀取並驗證，失敗回傳 ERROR；參數值放在 YAML
- 新增 controller plugin 時，plugin XML、`CMakeLists.txt`、`package.xml` 要同步修改
- BT 節點需要 ROS node 時，從 blackboard 的 `"node"` key 取得，不使用建構子注入（用 `graphen_bt_nodes/ros_node.hpp` 的 `get_ros_node()`）
- BT 節點的 XML port 不能命名為 `name`（v3 會當成節點的實例名稱）
- `rclcpp::Parameter::as_*_array()` 回傳的是參考：要先存到區域變數，不能直接用暫存值做 range-for
- IMU 驅動例外：使用自己的命名（`imu16460_*`、namespace `imu16460` / `imu16460_hardware`），不套用 `graphen_` 慣例；協議常數集中在 `protocol.hpp`，不寫魔術數字
- IMU 的 `read()` 只取 Driver 背景 thread 的最新樣本，不可阻塞、不可配置記憶體
- 待實機驗證的項目以 `// TODO(verify-on-hw):` 標記，不要自行假設為正確

## T170 / 靈巧手實機安全
- **未經使用者同意，不得以 `use_mock_hardware:=false` 啟動**（會對馬達送出指令）。WSL 開發機沒有 CAN / RS485：實機路徑只驗證 xacro 展開與建置

## IMU 實機安全
- **未經使用者同意，不得對實機送出會改變持久設定的指令**（存 FLASH、改鮑率、改坐標系、重啟）。開發與測試只做讀取；hardware plugin 不送任何命令
- WSL 開發機沒有接 IMU：實機路徑以 core 的 pty 測試驗證，或用 pty 假裝置啟動 `use_mock_imu:=false`

## Git
- **不要開分支、不要 commit**，git 操作由使用者手動處理。只修改工作目錄中的檔案，完成後說明改了哪些檔案。

## 測試注意事項（WSL）
- 每個 `ros2` CLI 指令都加 `timeout`；`ros2 topic pub` 要加 `-t`/`--once` 和 `-w`，避免等待 subscriber 而卡住。
- 背景啟動 launch 時用 `setsid`（非互動 shell 的背景 job 會忽略 SIGINT）；結束時對 `pgrep -x bt_runner` 等**確切的 process 名稱**送出 SIGINT，不要用 `pkill -f` 或 `pgrep -f`，否則會比對到自己的 shell。process 名稱最多 15 個字元，`pgrep -x` 要用截斷後的名稱：`ros2_control_no`、`robot_state_pub`（直接對 `ros2 launch` 的 PID 送 SIGINT 也可以，會一併結束子 process）。
- `ros2 param` 偶爾會回 "Node not found"（daemon 探索偏慢），重試即可。
- 測試完成後確認沒有殘留的 process（Groot 的 port 1666 被佔用時，下一個 `bt_runner` 會啟動失敗）。
- 只有 `imu16460_core` 有 GoogleTest（`colcon test --packages-select imu16460_core`）；其他套件要新增測試前先和使用者討論。
- 使用者可能已經開著 bringup / bt_runner：測試前先用 `ps` 確認，必要時設定不同的 `ROS_DOMAIN_ID` 隔離，且不要結束不是自己啟動的 process。
