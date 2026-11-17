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
