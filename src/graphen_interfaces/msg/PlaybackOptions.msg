# 所有播放類 action（SwitchMode / PlayChoreo / PlayBvh / RunPolicy）共用的 goal 選項

# 目前正在播的就是「同一個」時的處理方式（只影響這一次指令，不會留給之後的指令）：
#   ON_SAME_IGNORE              （預設）不重新開始
#   ON_SAME_RESTART             從頭重播（等同打斷目前的播放，受不可中斷的限制）
#   ON_SAME_RESTART_IF_FINISHED 目前這次播放已經結束（完成或失敗）時才重播，播放中則不重新開始；
#                               用來吃掉播放途中重複送出的指令
# 「同一個」的定義依類型而定：系統模式比名稱；編舞比名稱與內容；BVH 比名稱與播放選項；policy 比名稱與設定檔。
# 不是同一個時一律切換（除非目前在播的不可中斷）。
int8 ON_SAME_IGNORE=0
int8 ON_SAME_RESTART=1
int8 ON_SAME_RESTART_IF_FINISHED=2
int8 on_same 0

# false（預設）：切換後立即回傳成功。
# true：等到動作播完才回傳成功；等待期間被其他播放取代、動作失敗或 controller 停用時 abort。
#       持續性的動作（循環的 BVH、policy）不會結束。cancel 只會停止等待，不會停止或改變目前的動作
bool wait_until_finished false

# true：影子試播。在 controller 內部以目前的狀態為起點空跑整段動作（同一個演算法、同一套上限，
#       理想追蹤），不切換、不送命令，目前的動作不受影響。通過時 result.shadow_token 為試播的憑證，
#       軌跡以實際時間發布在 ~/shadow/joint_states。wait_until_finished 對試播不起作用（算完就回傳）。
#       目前只有 PlayChoreo 支援，其他 action 收到時 REJECT
bool dry_run false

# 正式播放時帶上試播回傳的 shadow_token：controller 確認內容（sha256）相同、未過期、
# 而且目前的起點和試播時相同（choreo.shadow.start_tolerance）才播放，否則 abort。
# 空字串 = 不檢查；controller 參數 choreo.shadow.require 為 true 時編舞一定要帶
string shadow_token
