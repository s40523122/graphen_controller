# controller 目前在播的東西（~/playback_state，transient_local，改變時發布）。
# 欄位意義同 PlaybackResult 的 mode / item：
#   mode  類型（choreo / bvh / policy）或系統模式名稱；尚未進入任何模式或 controller 未啟用時為空
#   item  項目名稱（舞曲、BVH 檔、policy 的名稱）；系統模式為空
string mode
string item
