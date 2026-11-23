# 視覺伺服的手臂末端目標（SwitchGroupController 訂閱 ~/pose_targets，交給 plugin 模式，例如 arm_servo）。
# 相機 → 軀幹的轉換由發送端完成（tf2），controller 不做 tf
std_msgs/Header header         # frame_id 必須等於 controller 參數 pose_targets.frame（預設 WAIST_P_S）
string arm                     # 必須是 pose_targets.names 之一
geometry_msgs/Pose pose        # 末端（tip_body + tip_offset）的位姿
float64 swivel                 # 手肘的 swivel（rad，定義同 SolveArmIk）；NaN = 不指定
