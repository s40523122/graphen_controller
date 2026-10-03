# graphen_ws

ROS 2 Humble colcon 工作區，包含兩層：ros2_control 控制層（`SwitchGroupController` → chainable `SubGroupController` → mock hardware），以及 BehaviorTree.CPP v3 行為層（`bt_runner` + plugin 節點）。兩層目前尚未連動。
架構、參數、設計決策見 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)；使用方式見 [README.md](README.md)。

## 套件
- `graphen_control_utils`：header-only 共用元件（`StateInterfacePublisher`）
- `graphen_controllers`：兩個 controller plugin，plugin 描述檔為 `graphen_controllers.xml`
- `graphen_description`：URDF / ros2_control
- `graphen_bringup`：`config/controllers.yaml`、`launch/bringup.launch.py`
- `graphen_bt_nodes`：BT 節點 plugin，全部在 `register_nodes.cpp` 註冊
- `graphen_bt`：`bt_runner`、`trees/`、`config/bt_runner.yaml`、`launch/bt.launch.py`

## 建置與執行
- `colcon build --symlink-install`；只改一個套件時加 `--packages-select <pkg>`
- 執行前先 `source install/setup.bash`（BT plugin 依賴 LD_LIBRARY_PATH 載入）
- `ros2 launch graphen_bringup bringup.launch.py`、`ros2 launch graphen_bt bt.launch.py`

## 慣例
- 套件名稱：`graphen_<功能>`
- namespace：控制層 `graphen::control`，行為層 `graphen::bt`
- controller plugin 名稱：`graphen_control/<Class>`
- header 放在 `include/<pkg>/`，include guard 為 `<PKG>__<FILE>_HPP_`
- controller 參數：在 `on_init` 用 `auto_declare` 宣告，在 `on_configure` 讀取並驗證，失敗回傳 ERROR；參數值放在 YAML
- 新增 controller plugin 時，plugin XML、`CMakeLists.txt`、`package.xml` 要同步修改
- BT 節點需要 ROS node 時，從 blackboard 的 `"node"` key 取得，不使用建構子注入
- `rclcpp::Parameter::as_*_array()` 回傳的是參考：要先存到區域變數，不能直接用暫存值做 range-for

## Git
- **不要開分支、不要 commit**，git 操作由使用者手動處理。只修改工作目錄中的檔案，完成後說明改了哪些檔案。

## 測試注意事項（WSL）
- 每個 `ros2` CLI 指令都加 `timeout`；`ros2 topic pub` 要加 `-t`/`--once` 和 `-w`，避免等待 subscriber 而卡住。
- 背景啟動 launch 時用 `setsid`（非互動 shell 的背景 job 會忽略 SIGINT）；結束時對 `pgrep -x bt_runner` 等**確切的 process 名稱**送出 SIGINT，不要用 `pkill -f` 或 `pgrep -f`，否則會比對到自己的 shell。
- `ros2 param` 偶爾會回 "Node not found"（daemon 探索偏慢），重試即可。
- 測試完成後確認沒有殘留的 process（Groot 的 port 1666 被佔用時，下一個 `bt_runner` 會啟動失敗）。
- 目前沒有自動化測試框架，要新增前先和使用者討論。
