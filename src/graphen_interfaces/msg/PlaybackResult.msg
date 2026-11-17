# 所有播放類 action 共用的 result

# 被拒絕（例如目前在播的不可中斷）、載入失敗、被取代或動作失敗時 success 為 false，message 說明原因
bool success
string message

# 回傳當下實際在播的東西：
#   mode  類型（choreo / bvh / policy）或系統模式名稱（例如 stop）；尚未進入任何模式時為空
#   item  項目名稱（舞曲、BVH 檔、policy 的名稱）；系統模式為空
string mode
string item

# 回傳當下的 state interface 快照（名稱與數值一一對應）
string[] interface_names
float64[] values
