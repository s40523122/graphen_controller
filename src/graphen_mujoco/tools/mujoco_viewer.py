"""顯示機器人的姿態（MuJoCo 模型，只顯示，不做物理），並提供手臂示教的拖曳與 IK。

姿態來源（--source）：
  udp（預設）  MujocoSystem（ros2_control_node 內，hardware:=mujoco / mujoco_pd）以 UDP 把
               [time, qpos(nq), actuator_force(nu)] 送到 127.0.0.1:<port>
  ros          訂閱 /joint_states（--joint-states-topic），給 hardware:=mock / real / real_pd 使用；
               必須指定 --model，基座固定在模型的初始姿態，effort 當作力矩顯示
載入同一份 MJCF，收到後設定 qpos、mj_forward，再更新視窗；
各致動器的力矩以文字疊加在畫面右上角（力矩 / 上限 與百分比，達 warn_ratio 以上標 "!"），
並在關節上畫箭頭：沿轉軸、依右手定則指向力矩方向，長度與顏色（綠 → 紅）依 力矩 / 上限。
箭頭的起點在關節中心、會被機身遮住，所以顯示箭頭時機身改為半透明（T 鍵可以再切換）。
按 End 鍵送出 "reset" 到 <port>+1，模擬回到初始姿態（只有 udp）；Home 鍵切換文字，PageUp 鍵切換箭頭。
手腕（--wrist-bodies，預設兩手的 L/R_WRIST_R_S，即 arm_ik.yaml 的 tip_body）的座標軸以箭頭顯示
（x 紅、y 綠、z 藍；tcp_jog 的 FRAME_TOOL 就是沿 / 繞這組軸），Pause 鍵切換，模擬與示教模式都會顯示。

幽靈（--ghost-topic）：把任意 sensor_msgs/JointState topic 的姿態以半透明的第二個機器人疊在畫面上
（例如 controller 影子試播的 /switch_group_controller/shadow/joint_states），超過 --ghost-timeout 秒
沒有收到就隱藏。沒有列出的 joint 和基座沿用目前顯示的姿態。

示教模式：controller（--controller）的 ~/playback_state 的 mode 等於 --teach-mode（預設 teach，
controllers.yaml 的 hold 模式）時自動進入，離開該模式就結束（--teach-always：不看 controller，一直示教）。
  - viewer 只負責拖曳與 IK，不記錄、不匯出、不送指令：示教的姿態發布在 /mujoco_viewer_teach/pose
    （sensor_msgs/JointState，全身的 joint，joint 與順序同 /joint_states，velocity / effort 為 0，
    transient_local），由外部程式擷取、組成編舞，
    再以 PlayChoreo 的影子試播（options.dry_run）檢查後播放
  - IK 與碰撞檢查由 graphen_arm_ik 的 arm_ik_server 計算（與 ROS 指令、視覺伺服共用同一份演算法與
    arm_ik.yaml），viewer 是它的 client：要能 import rclpy（先 source install/setup.bash，以 python 3.10
    的 venv 安裝 mujoco==3.11.0、glfw、pyyaml、numpy<2，見 README），並在同一個 ROS_DOMAIN_ID 啟動 server
    （bringup 預設會啟動）。沒有 rclpy 時示教、ros 來源與幽靈都停用，udp 顯示不受影響
  - 進入時停止套用收到的姿態（以當下畫面，也就是機器人目前的姿態為起點），兩手各有一個
    手腕目標（紅 = 右手、藍 = 左手），雙擊選取後 Ctrl+右鍵拖曳平移、Ctrl+左鍵拖曳旋轉，
    手臂跟隨：wrist 位置 + 姿態對準目標（目標以 arm_ik.yaml 的 root_body 座標系送出）
  - 也可以直接指定關節角：發布到 /mujoco_viewer_teach/set_pose（sensor_msgs/JointState，只填要改的 joint，
    格式同 /joint_states）。只套用列出的 joint，超出限位時截斷並印出警告，手腕目標跟著移到新的位置；
    之後可以繼續用滑鼠微調。不在示教時收到的會被忽略
  - PageDown 切換要調整 swivel 的手臂（拖曳哪個目標就切到哪隻手）。F9 / F10 調整手肘的 swivel
    （繞「肩 → 腕」軸，軀幹座標的絕對角度），wrist 不動；碰到 joint 範圍時畫面顯示實際值與要求值的差
  - 解的姿態有碰撞時目標變黃，畫面列出碰撞的 body 對（只警告，仍然可以拖曳）
  - 結束示教後恢復顯示收到的姿態。示教只改畫面，不會送任何指令給模擬或機器人
  - 需要 MjSpec（MuJoCo 3.2 以上）在載入時加入目標的 mocap body，MJCF 檔本身不修改

udp 來源沒有指定 --model 時向 MujocoSystem 查詢它載入的 MJCF（送 "model?" 到 <port>+1，等到 bringup
啟動為止），所以模型會跟著 bringup 的 robot / mujoco_fixed_base。回覆的路徑在這台機器上不存在時
（bringup 在容器內、viewer 在主機），改用預設的 fb70 t170_free.xml；這種情況要自己指定 --model。

需要 mujoco 與 glfw，通常由 mujoco_viewer.sh 啟動：
  bash mujoco_viewer.sh                         # MuJoCo 硬體：模型向 MujocoSystem 查詢、port 9870
  # mock / real：訂閱 /joint_states；切到 teach 模式時開始示教，影子試播以幽靈顯示
  PY=~/teach_env/bin/python bash mujoco_viewer.sh --source ros \
      --model ../../robot_description/ti5_170a_graphen/mjcf/ti5_170a.xml \
      --ghost-topic /switch_group_controller/shadow/joint_states
  # 不開 bringup 示教：另外啟動 arm_ik_server（模型要和 viewer 的一致），加 --teach-always
  ros2 run graphen_arm_ik arm_ik_server --ros-args \
      -p arm_ik_config:=package://ti5_170a_graphen/config/arm_ik.yaml
  PY=~/teach_env/bin/python bash mujoco_viewer.sh --teach-always \
      --model ../../robot_description/ti5_170a_graphen/mjcf/ti5_170a.xml
"""
import argparse
import queue
import socket
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

HERE = Path(__file__).resolve().parent
# 查不到模型時的預設：fb70_description 套件（src/robot_description/fb70）的浮動基座 MJCF
FB70_MJCF = HERE.parents[1] / "robot_description" / "fb70" / "mjcf"
MODEL_QUERY = b"model?"
MODEL_REPLY_PREFIX = b"model "
MODEL_QUERY_PERIOD = 1.0   # 重新查詢的間隔（s）
KEY_END = 269
KEY_HOME = 268
KEY_PAGE_UP = 266
KEY_WRIST_AXES = 284     # Pause
# 示教模式的按鍵：避開 MuJoCo viewer 已使用的（F1–F5、PageUp、Space、方向鍵、[ ]、字母與數字）
KEY_TEACH_SWITCH_ARM = 267   # PageDown
KEY_SWIVEL_MINUS = 298   # F9
KEY_SWIVEL_PLUS = 299    # F10
TEACH_KEYS = {KEY_TEACH_SWITCH_ARM, KEY_SWIVEL_MINUS, KEY_SWIVEL_PLUS}
TARGET_BODY = "teach_target_{}"
TARGET_RGBA = {"L": [0.2, 0.4, 1.0, 0.6], "R": [1.0, 0.25, 0.2, 0.6]}
TARGET_SIZE = 0.03        # 目標方塊的半邊長（m）
TARGET_AXIS_LENGTH = 0.08  # 目標上 x / y / z 軸標示的長度（m）
ARROW_MIN_LENGTH = 1e-3   # 比這個短的箭頭不畫（m）
AXIS_COLORS = ([1.0, 0.0, 0.0, 1.0], [0.0, 1.0, 0.0, 1.0], [0.0, 0.0, 1.0, 1.0])   # x / y / z
GHOST_RGBA = np.array([0.3, 0.9, 1.0, 0.35], dtype=np.float32)   # 幽靈的顏色（半透明）


def actuator_labels(m):
    """致動器顯示名稱：驅動 joint 的致動器用 joint 名稱，其他用致動器名稱。"""
    labels = []
    for i in range(m.nu):
        if m.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT:
            labels.append(m.joint(m.actuator_trnid[i, 0]).name)
        else:
            labels.append(m.actuator(i).name)
    return labels


def actuator_limits(m):
    """各致動器的力矩上限；沒有上限時為 NaN（只顯示數值，不算百分比）。

    forcerange 優先；motor 致動器（gain 固定、沒有 bias）沒有 forcerange 時，由 ctrlrange × gear × gain 推得。
    """
    limits = np.full(m.nu, np.nan)
    for i in range(m.nu):
        if m.actuator_forcelimited[i]:
            limits[i] = np.abs(m.actuator_forcerange[i]).max()
        elif (m.actuator_ctrllimited[i]
              and m.actuator_gaintype[i] == mujoco.mjtGain.mjGAIN_FIXED
              and m.actuator_biastype[i] == mujoco.mjtBias.mjBIAS_NONE):
            limits[i] = (np.abs(m.actuator_ctrlrange[i]).max()
                         * abs(m.actuator_gear[i, 0] * m.actuator_gainprm[i, 0]))
    return limits


def actuator_hinges(m):
    """各致動器驅動的 hinge joint id；不是驅動 hinge 的致動器為 -1（不畫箭頭）。"""
    hinges = np.full(m.nu, -1)
    for i in range(m.nu):
        if m.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT:
            joint = m.actuator_trnid[i, 0]
            if m.jnt_type[joint] == mujoco.mjtJoint.mjJNT_HINGE:
                hinges[i] = joint
    return hinges


def draw_torque_arrows(scn, d, hinges, limits, forces, args):
    """在 user_scn 畫各關節的力矩箭頭（呼叫前 d 已經 mj_forward）。

    有上限的軸長度為 arrow_length × |力矩| / 上限；沒有上限的軸以 arrow_scale（m / N·m）換算、灰色顯示。
    """
    scn.ngeom = 0
    for joint, limit, force in zip(hinges, limits, forces):
        if joint < 0 or scn.ngeom >= scn.maxgeom:
            continue
        if np.isfinite(limit) and limit > 0:
            ratio = min(abs(force) / limit, 1.0)
            length = args.arrow_length * ratio
            rgba = np.array([ratio, 1.0 - ratio, 0.0, 0.9])
        else:
            length = args.arrow_scale * abs(force)
            rgba = np.array([0.6, 0.6, 0.6, 0.9])
        if length < ARROW_MIN_LENGTH:
            continue
        start = d.xanchor[joint]
        end = start + np.sign(force) * length * d.xaxis[joint]
        geom = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3),
                            np.zeros(9), rgba.astype(np.float32))
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, args.arrow_width, start, end)
        scn.ngeom += 1


def draw_frame_axes(scn, d, bodies, length, width):
    """在 user_scn 畫 bodies 的座標軸（接在已有的 geom 之後；呼叫前 d 已經更新運動學）."""
    for body in bodies:
        origin = d.xpos[body]
        rotation = d.xmat[body].reshape(3, 3)
        for axis, color in enumerate(AXIS_COLORS):
            if scn.ngeom >= scn.maxgeom:
                return
            geom = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3),
                                np.zeros(9), np.array(color, dtype=np.float32))
            mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, width, origin,
                                 origin + length * rotation[:, axis])
            scn.ngeom += 1


def torque_text(labels, limits, forces, warn_ratio):
    names, values = [], []
    for label, limit, force in zip(labels, limits, forces):
        if np.isfinite(limit) and limit > 0:
            ratio = abs(force) / limit
            mark = " !" if ratio >= warn_ratio else ""
            values.append(f"{force:7.1f} / {limit:5.0f} ({ratio * 100:3.0f}%){mark}")
        else:
            values.append(f"{force:7.1f}")
        names.append(label)
    return "\n".join(names), "\n".join(values)


def query_model(sock, port):
    """向 MujocoSystem 查詢載入的 MJCF 路徑；bringup 還沒啟動時一直等。

    回覆和姿態封包走同一個 socket，以開頭的 MODEL_REPLY_PREFIX 分辨。
    """
    print(f"asking MujocoSystem on 127.0.0.1:{port + 1} for its model "
          "(waiting for bringup; pass --model to skip)")
    try:
        while True:
            sock.sendto(MODEL_QUERY, ("127.0.0.1", port + 1))
            deadline = time.monotonic() + MODEL_QUERY_PERIOD
            while (remaining := deadline - time.monotonic()) > 0:
                sock.settimeout(remaining)
                try:
                    packet = sock.recv(65536)
                except socket.timeout:
                    break
                except ConnectionError:
                    # 對方的 port 還沒開（ICMP unreachable）：等到這一輪結束再問
                    time.sleep(max(0.0, deadline - time.monotonic()))
                    break
                if packet.startswith(MODEL_REPLY_PREFIX):
                    return packet[len(MODEL_REPLY_PREFIX):].decode()
    finally:
        sock.setblocking(False)


def load_model(path):
    """載入 MJCF，並加入示教用的兩個手腕目標（mocap body，沒有 joint，nq / nu 不變）。

    回傳 (model, 是否支援示教)。沒有 MjSpec（MuJoCo 3.2 以前）時照原樣載入、不支援示教。
    """
    if not hasattr(mujoco, "MjSpec"):
        return mujoco.MjModel.from_xml_path(path), False
    spec = mujoco.MjSpec.from_file(path)
    for side, rgba in TARGET_RGBA.items():
        body = spec.worldbody.add_body(name=TARGET_BODY.format(side), mocap=True)
        # contype / conaffinity = 0：只顯示，不參與碰撞
        body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[TARGET_SIZE] * 3, rgba=rgba,
                      contype=0, conaffinity=0)
        for axis, color in enumerate(AXIS_COLORS):
            end = np.zeros(3)
            end[axis] = TARGET_AXIS_LENGTH
            body.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[0.0, 0.0, 0.0, *end],
                          size=[0.004, 0.0, 0.0], rgba=color, contype=0, conaffinity=0)
    return spec.compile(), True


class TextOverlay:
    """viewer 疊字：內容有變才送出。

    passive viewer 的 set_texts() 會等 render thread 取走文字才回傳；視窗關閉後 render thread 已結束，
    呼叫會永遠卡住，程式就關不掉。所以只在內容改變、且視窗還開著時才呼叫，縮小這個時間窗。
    """

    def __init__(self, viewer):
        self.viewer = viewer
        self.texts = None

    def show(self, texts):
        if texts == self.texts or not self.viewer.is_running():
            return
        self.texts = texts
        self.viewer.set_texts(texts)




ARM_NAMES = {"L": "left", "R": "right"}     # arm_ik.yaml 的手臂名稱
COLLISION_RGBA = [1.0, 0.9, 0.0, 0.9]         # 解的姿態有碰撞時目標方塊的顏色
# 一個請求等待回應的時間（s）。Humble 的 service 剛被發現時，server 可能還沒發現 client 的回應端，
# 回應會被丟掉且不重送（WSL 上探索慢，較常發生），所以逾時就重送
REQUEST_TIMEOUT = 1.0
FORWARD_ATTEMPTS = 5                           # forward（進入示教時）最多送幾次
RETRY_IMPROVEMENT = 0.99   # 沒收斂時，誤差要降到上次的這個比例以下才以新的解為起點繼續求解
TEACH_RETRY_PERIOD = 2.0   # 應該示教但還無法進入（arm_ik_server 還沒找到等）時，重試的間隔（s）
SPIN_BATCH = 20            # 主迴圈每一輪最多處理幾個 ROS callback（不讓大量訊息卡住畫面）
NODE_NAME = "mujoco_viewer_teach"


class RosLink:
    """viewer 的 ROS 連線（一個 node，主迴圈每一輪 spin()，不另開 thread）。

    rclpy 在建立時才 import：沒有 ROS 環境時丟出 ImportError，viewer 只保留 udp 顯示。
      - <controller>/playback_state：controller 目前的模式（決定是否示教）
      - joint_states_topic：--source ros 時顯示的姿態；示教姿態的 joint 與順序也以它為準
      - ghost_topic：幽靈的姿態
      - ~/pose（/mujoco_viewer_teach/pose）：示教的全身姿態（格式同 /joint_states）
      - ~/set_pose（/mujoco_viewer_teach/set_pose）：示教中直接指定關節角（只填要改的 joint）
    callback 只保存最新的訊息，由主迴圈在持有 viewer lock 時套用。
    """

    def __init__(self, args):
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from graphen_interfaces.msg import PlaybackState
        from sensor_msgs.msg import JointState
        from rclpy.signals import SignalHandlerOptions
        self.rclpy = rclpy
        self.JointState = JointState
        if not rclpy.ok():
            # 不讓 rclpy 接管 SIGINT / SIGTERM：它只會關閉 context，主迴圈不會結束，視窗就關不掉
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node(NODE_NAME)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode = None            # 收到 playback_state 之前為 None
        self.joint_state = None     # 最新的 /joint_states（--source ros 時顯示用，套用後清除）
        self.joint_state_reference = None   # 最新的 /joint_states（示教姿態的 joint 順序與模型外的 joint）
        self.show_joint_states = args.source == "ros"
        self.ghost = None           # 最新的幽靈姿態（套用後清除）
        self.set_pose = []          # 收到的 ~/set_pose（示教中依序套用後清除）
        self.ghost_stamp = 0.0      # 最後一次收到幽靈的時間（monotonic）
        self.subscriptions = []
        if args.controller:
            self.subscriptions.append(self.node.create_subscription(
                PlaybackState, f"{args.controller.rstrip('/')}/playback_state", self._on_mode, latched))
        # udp 來源也訂閱：示教姿態要和 /joint_states 的 joint 與順序一致（MuJoCo 硬體也有 broadcaster）
        self.subscriptions.append(self.node.create_subscription(
            JointState, args.joint_states_topic, self._on_joint_state, 10))
        if args.ghost_topic:
            self.subscriptions.append(self.node.create_subscription(
                JointState, args.ghost_topic, self._on_ghost, 10))
        self.pose_publisher = self.node.create_publisher(JointState, "~/pose", latched)
        self.subscriptions.append(self.node.create_subscription(
            JointState, "~/set_pose", self.set_pose.append, 10))

    def _on_mode(self, msg):
        self.mode = msg.mode

    def _on_joint_state(self, msg):
        self.joint_state_reference = msg
        if self.show_joint_states:
            self.joint_state = msg

    def _on_ghost(self, msg):
        self.ghost = msg
        self.ghost_stamp = time.monotonic()

    def spin(self):
        for _ in range(SPIN_BATCH):
            self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def publish_pose(self, names, positions):
        """格式同 /joint_states：name / position / velocity / effort 等長；示教的是靜態姿態，速度與力矩為 0。"""
        count = len(names)
        msg = self.JointState(name=list(names), position=[float(p) for p in positions],
                              velocity=[0.0] * count, effort=[0.0] * count)
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.pose_publisher.publish(msg)

    def close(self):
        self.node.destroy_node()


class JointMap:
    """sensor_msgs/JointState → 模型的 qpos / 致動器；名稱對應在第一次遇到時建立並快取。

    模型中沒有的 joint（例如靈巧手）忽略；訊息中沒有的 joint 保持原值。
    """

    def __init__(self, m):
        self.qpos_adr = {m.joint(j).name: m.jnt_qposadr[j] for j in range(m.njnt)
                         if m.jnt_type[j] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)}
        # 各致動器驅動的 joint 名稱（不是驅動 joint 的致動器為 None）
        self.actuator_joints = [m.joint(m.actuator_trnid[i, 0]).name
                                if m.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT else None
                                for i in range(m.nu)]
        self.cache_key = None
        self.cache = None

    def _mapping(self, names):
        key = tuple(names)
        if key != self.cache_key:
            pairs = [(i, self.qpos_adr[n]) for i, n in enumerate(names) if n in self.qpos_adr]
            index = {n: i for i, n in enumerate(names)}
            actuators = [index.get(j) if j is not None else None for j in self.actuator_joints]
            self.cache_key, self.cache = key, (pairs, actuators)
        return self.cache

    def apply(self, qpos, msg):
        """把 msg.position 寫入 qpos；回傳對應到的 joint 數。"""
        pairs, _ = self._mapping(msg.name)
        count = 0
        for i, adr in pairs:
            if i < len(msg.position):
                qpos[adr] = msg.position[i]
                count += 1
        return count

    def forces(self, msg):
        """msg.effort 依致動器排列；沒有 effort 時回傳 None，沒有對應的致動器為 0。"""
        if not msg.effort:
            return None
        _, actuators = self._mapping(msg.name)
        return np.array([msg.effort[i] if i is not None and i < len(msg.effort) else 0.0
                         for i in actuators])


class IkClient:
    """graphen_arm_ik 的 arm_ik_server（~/solve、~/forward）的非同步 client（共用 RosLink 的 node）。

    主迴圈每一輪 spin RosLink 的 node 處理回應，不會阻塞畫面。
    """

    def __init__(self, ros, server):
        from graphen_interfaces.srv import ArmForward, SolveArmIk
        self.rclpy = ros.rclpy
        self.node = ros.node
        self.SolveArmIk, self.ArmForward, self.JointState = SolveArmIk, ArmForward, ros.JointState
        self.solve_client = self.node.create_client(SolveArmIk, f"{server}/solve")
        self.forward_client = self.node.create_client(ArmForward, f"{server}/forward")
        self.server = server

    def ready(self):
        """不等待：兩個 service 都已經被發現。"""
        return self.solve_client.service_is_ready() and self.forward_client.service_is_ready()

    def state(self, names, positions):
        return self.JointState(name=list(names), position=[float(p) for p in positions])

    def forward(self, arm, state):
        """同步呼叫（只在進入示教時使用）；沒有回應就重送（只讀取，重送沒有副作用），都失敗回傳 None。"""
        request = self.ArmForward.Request(arm=arm, state=state)
        for _ in range(FORWARD_ATTEMPTS):
            future = self.forward_client.call_async(request)
            self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=REQUEST_TIMEOUT)
            if future.done():
                return future.result()
        return None

    def solve_async(self, arm, position, quat_wxyz, swivel, seed, state, frame):
        # local：拖曳時解要連續，不重試其他起點（不會突然跳到另一個分支的姿態）
        request = self.SolveArmIk.Request(arm=arm, swivel=float(swivel), seed=[float(q) for q in seed],
                                          state=state, local=True)
        request.target.header.frame_id = frame
        p, q = request.target.pose.position, request.target.pose.orientation
        p.x, p.y, p.z = (float(v) for v in position)
        q.w, q.x, q.y, q.z = (float(v) for v in quat_wxyz)
        return self.solve_client.call_async(request)


def _quat_mul(a, b):
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, a, b)
    return out


def _quat_neg(a):
    out = np.zeros(4)
    mujoco.mju_negQuat(out, a)
    return out


class Teacher:
    """viewer 的示教模式：手腕目標（mocap）的拖曳、IK（arm_ik_server）、swivel 與碰撞警示。

    IK 與碰撞檢查都在 arm_ik_server（graphen_arm_ik，與指令 / 伺服共用同一份演算法與 arm_ik.yaml），
    viewer 只送目標與自己畫面上的全身姿態（state），收到解之後套用到畫面，並把全身的姿態發布在
    /mujoco_viewer_teach/pose。一次只有一個請求在途中，拖曳中只送最新的目標，不會排隊。
    所有方法都在主迴圈、持有 viewer lock 時呼叫。
    """

    def __init__(self, m, args, ros):
        self.m = m
        self.ros = ros
        self.client = IkClient(ros, args.arm_ik_server)
        bodies = {side: m.body(TARGET_BODY.format(side)).id for side in TARGET_RGBA}
        self.side_of_body = {body: side for side, body in bodies.items()}
        self.mocap = {side: m.body_mocapid[body] for side, body in bodies.items()}
        self.box_geom = {side: m.body_geomadr[body] for side, body in bodies.items()}
        self.geoms = [g for body in bodies.values()
                      for g in range(m.body_geomadr[body], m.body_geomadr[body] + m.body_geomnum[body])]
        self.rgba = m.geom_rgba[self.geoms].copy()
        # 送給 server 的全身姿態：模型中所有 hinge / slide joint
        self.state_joints = [(m.joint(j).name, m.jnt_qposadr[j]) for j in range(m.njnt)
                             if m.jnt_type[j] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)]
        self.qpos_of = dict(self.state_joints)
        self.swivel_step = np.deg2rad(args.swivel_step)
        self.active = False
        self.side = "R"
        self.frame = None         # 目標的座標系（arm_ik.yaml 的 root_body）
        self.root_body = -1
        self.joints = {}          # side → 7 個 joint 名稱
        self.qpos_adr = {}        # side → 7 個 qpos 位址
        self.swivel = {}          # side → 要求的 swivel（rad）；進入時為目前值，F9 / F10 調整
        self.pending = None       # (future, side, 送出的時間)
        self.sent = {}            # side → 上次送出的 (pos, quat, swivel)
        self.response = {}        # side → 最新的 SolveArmIk 回應
        self.retry = {}           # side → 沒收斂但誤差仍在下降，下一輪以新的解繼續求解
        self.collisions = {}      # side → 目前姿態中與這隻手臂有關的碰撞（solve 或 forward 的結果）
        self.set_visible(False)

    def set_visible(self, visible):
        self.m.geom_rgba[self.geoms] = self.rgba
        if not visible:
            self.m.geom_rgba[self.geoms, 3] = 0.0

    def current_state(self, d):
        names, adrs = zip(*self.state_joints)
        return self.client.state(names, d.qpos[list(adrs)])

    def root_pose(self, d):
        return d.xpos[self.root_body].copy(), d.xquat[self.root_body].copy()

    def to_world(self, d, pos, quat):
        root_pos, root_quat = self.root_pose(d)
        world = np.zeros(3)
        mujoco.mju_rotVecQuat(world, pos, root_quat)
        return world + root_pos, _quat_mul(root_quat, quat)

    def to_root(self, d, pos, quat):
        root_pos, root_quat = self.root_pose(d)
        inverse = _quat_neg(root_quat)
        local = np.zeros(3)
        mujoco.mju_rotVecQuat(local, pos - root_pos, inverse)
        return local, _quat_mul(inverse, quat)

    def publish_pose(self, d):
        """發布全身的示教姿態，joint 與順序同 /joint_states。

        收到過 /joint_states 時沿用它的 joint 與順序：模型中的 joint 取畫面的姿態（手臂為 IK 的解，
        其他為進入示教時的姿態），模型外的 joint（例如靈巧手）沿用 /joint_states 的最新值。
        還沒收到時改為模型中所有 hinge / slide joint（模型的順序）。
        """
        reference = self.ros.joint_state_reference
        if reference is not None and reference.name:
            names = list(reference.name)
            positions = [d.qpos[self.qpos_of[n]] if n in self.qpos_of
                         else (reference.position[i] if i < len(reference.position) else float("nan"))
                         for i, n in enumerate(names)]
        else:
            names = [n for n, _ in self.state_joints]
            positions = [d.qpos[adr] for _, adr in self.state_joints]
        self.ros.publish_pose(names, positions)

    def enter(self, d):
        """以 d（目前畫面的姿態）為起點進入示教；arm_ik_server 還沒找到或失敗時回傳 False（之後再試）。"""
        if not self.client.ready():
            print(f"teach mode: waiting for {self.client.server} (start it in the same ROS_DOMAIN_ID: "
                  "ros2 launch graphen_bringup bringup.launch.py, or ros2 run graphen_arm_ik "
                  "arm_ik_server --ros-args -p arm_ik_config:=<arm_ik.yaml>)")
            return False
        if not self.sync_targets(d):
            return False
        self.active = True
        self.pending = None
        self.sent = {side: self.target(d, side) for side in ARM_NAMES}
        self.response = {}
        self.retry = {}
        self.set_visible(True)
        for side in ARM_NAMES:   # set_visible 會重設顏色：補上碰撞的標示
            self.paint(side, bool(self.collisions.get(side)))
        self.publish_pose(d)
        print("teach mode on: double-click a wrist target, Ctrl+right drag = move, "
              "Ctrl+left drag = rotate, or publish joint angles on "
              f"/{NODE_NAME}/set_pose; the pose (same joints as /joint_states) is published on "
              f"/{NODE_NAME}/pose")
        return True

    def sync_targets(self, d):
        """以 d 的姿態向 arm_ik_server 查詢兩手的末端位姿（~/forward），把手腕目標移過去。

        同時取得手臂的 joint、目標的座標系、目前的 swivel 與碰撞（目標變黃）。失敗時回傳 False。
        """
        mujoco.mj_forward(self.m, d)
        state = self.current_state(d)
        for side, arm in ARM_NAMES.items():
            response = self.client.forward(arm, state)
            if response is None or not response.success:
                print(f"teach mode: forward for arm '{arm}' failed: "
                      f"{'no response' if response is None else response.message}")
                return False
            try:
                adrs = [self.m.jnt_qposadr[self.m.joint(j).id] for j in response.joint_names]
                root = self.m.body(response.pose.header.frame_id).id
            except KeyError as e:
                print(f"teach mode: the viewer model does not match the arm_ik_server model ({e})")
                return False
            self.joints[side], self.qpos_adr[side] = list(response.joint_names), adrs
            self.frame, self.root_body = response.pose.header.frame_id, root
            self.swivel[side] = response.swivel
            self.collisions[side] = list(response.collisions)
            self.paint(side, bool(response.collisions))
            p, q = response.pose.pose.position, response.pose.pose.orientation
            pos, quat = self.to_world(d, np.array([p.x, p.y, p.z]), np.array([q.w, q.x, q.y, q.z]))
            d.mocap_pos[self.mocap[side]], d.mocap_quat[self.mocap[side]] = pos, quat
        # 更新目標的 xpos：viewer 開始拖曳時以它作為起點
        mujoco.mj_forward(self.m, d)
        return True

    def apply_joints(self, d, msg):
        """套用 ~/set_pose 指定的關節角（只改列出的 joint，超出限位時截斷），之後手腕目標跟著移動。

        進行中的 IK 請求作廢（它的解會蓋掉指定的角度），目標與送出紀錄改為新的姿態，不會再觸發求解。
        """
        if len(msg.position) < len(msg.name):
            print(f"set_pose: {len(msg.name)} names but {len(msg.position)} positions, "
                  "the names without a position are ignored")
        applied, clamped, skipped = [], [], []
        for name, value in zip(msg.name, msg.position):
            adr = self.qpos_of.get(name)
            if adr is None or not np.isfinite(value):
                skipped.append(name)
                continue
            joint = self.m.joint(name).id
            target = float(value)
            if self.m.jnt_limited[joint]:
                low, high = self.m.jnt_range[joint]
                if not low <= target <= high:
                    clamped.append(f"{name} {target:+.3f} -> [{low:+.3f}, {high:+.3f}]")
                    target = float(np.clip(target, low, high))
            d.qpos[adr] = target
            applied.append(name)
        if skipped:
            print(f"set_pose: ignored (not in the model, or not a finite number): {', '.join(skipped)}")
        if clamped:
            print(f"set_pose: clamped to the joint limits: {'; '.join(clamped)}")
        if not applied:
            return
        self.pending = None
        self.response = {}
        self.retry = {}
        if not self.sync_targets(d):
            print("set_pose: the pose is applied, but the wrist targets could not be updated")
        self.sent = {side: self.target(d, side) for side in ARM_NAMES}
        self.publish_pose(d)
        colliding = [ARM_NAMES[side] for side in ARM_NAMES if self.collisions.get(side)]
        print(f"set_pose: applied {len(applied)} joint(s)"
              + (f"; collisions on the {', '.join(colliding)} arm" if colliding else ""))

    def exit(self):
        self.active = False
        self.pending = None
        self.set_visible(False)
        print("teach mode off")

    def target(self, d, side):
        pos, quat = self.to_root(d, d.mocap_pos[self.mocap[side]].copy(),
                                 d.mocap_quat[self.mocap[side]].copy())
        return pos, quat, self.swivel[side]

    def handle_key(self, key):
        if key == KEY_TEACH_SWITCH_ARM:
            self.side = "L" if self.side == "R" else "R"
        elif key in (KEY_SWIVEL_MINUS, KEY_SWIVEL_PLUS):
            swivel = self.swivel[self.side]
            if not np.isfinite(swivel):
                print("elbow swivel is undefined in this pose (shoulder-wrist axis along the reference)")
                return
            step = self.swivel_step if key == KEY_SWIVEL_PLUS else -self.swivel_step
            self.swivel[self.side] = float(np.arctan2(np.sin(swivel + step), np.cos(swivel + step)))

    def update(self, d, pert):
        """套用拖曳、收送 IK；畫面需要更新時回傳 True。回應由主迴圈的 RosLink.spin() 處理。"""
        changed = False
        dragging = pert.active and pert.select in self.side_of_body
        if dragging:
            # flg_paused = 0：只移動 mocap body（目標），不動機器人本體
            mujoco.mjv_applyPerturbPose(self.m, d, pert, 0)
            self.side = self.side_of_body[pert.select]
            changed = True

        if (self.pending is not None and not self.pending[0].done()
                and time.monotonic() - self.pending[2] > REQUEST_TIMEOUT):
            # 回應遺失：放棄這個請求（之後才到的回應不會被讀取），下面以最新的目標重送
            self.sent.pop(self.pending[1], None)
            self.pending = None
        if self.pending is not None and self.pending[0].done():
            future, side, _ = self.pending
            self.pending = None
            response = future.result()
            if response is not None and len(response.positions) == len(self.qpos_adr[side]):
                previous = self.response.get(side)
                error = response.position_error + response.orientation_error
                self.retry[side] = not response.success and not response.collisions and (
                    previous is None or
                    error < RETRY_IMPROVEMENT * (previous.position_error + previous.orientation_error))
                self.response[side] = response
                self.collisions[side] = list(response.collisions)
                # 沒有完全收斂也套用（盡量接近目標），誤差與原因顯示在畫面上
                d.qpos[self.qpos_adr[side]] = response.positions
                self.paint(side, bool(response.collisions))
                mujoco.mj_forward(self.m, d)
                self.publish_pose(d)
                changed = True

        if self.pending is None:
            side = self.side
            target = self.target(d, side)
            last = self.sent.get(side)
            moved = (last is None or not np.allclose(last[0], target[0], atol=1e-5)
                     or not np.allclose(last[1], target[1], atol=1e-6)
                     or not np.array_equal([last[2]], [target[2]], equal_nan=True))
            # 一次求解的迭代次數有限（例如從伸直的手臂出發）：誤差還在下降就繼續
            if moved or self.retry.get(side, False):
                self.retry[side] = False
                seed = d.qpos[self.qpos_adr[side]]
                future = self.client.solve_async(ARM_NAMES[side], target[0], target[1], target[2], seed,
                                                 self.current_state(d), self.frame)
                self.pending = (future, side, time.monotonic())
                self.sent[side] = target
        return changed

    def paint(self, side, colliding):
        box = self.box_geom[side]
        index = self.geoms.index(box)
        self.m.geom_rgba[box] = COLLISION_RGBA if colliding else self.rgba[index]

    def overlay(self):
        response = self.response.get(self.side)
        if response is None:
            error, status = "-", "-"
        else:
            error = (f"{response.position_error * 1e3:.1f} mm / "
                     f"{np.rad2deg(response.orientation_error):.2f} deg")
            status = response.message.split(" [")[0]
        found = self.collisions.get(self.side)
        if found is None:
            collisions = "-"
        else:
            collisions = ", ".join(found[:3]) or "none"
            if len(found) > 3:
                collisions += f" (+{len(found) - 3})"
        wanted = self.swivel[self.side]
        actual = response.swivel if response is not None else wanted
        swivel = (f"{np.rad2deg(actual):+.1f} deg (want {np.rad2deg(wanted):+.1f})"
                  if np.isfinite(actual) and np.isfinite(wanted) else "undefined")
        names = "Teach\nArm\nElbow swivel\nWrist error\nIK\nCollisions"
        values = (f"on  (pose on /{NODE_NAME}/pose)\n"
                  f"{ARM_NAMES[self.side]}  (PageDown)\n"
                  f"{swivel}  (F9 / F10)\n"
                  f"{error}\n"
                  f"{status}\n"
                  f"{collisions}")
        return names, values


class Ghost:
    """以半透明的第二個機器人顯示另一組姿態（第二份 MjData + mjv_addGeoms，不修改模型）。"""

    def __init__(self, m, joint_map, hidden_geoms, timeout):
        self.m = m
        self.d = mujoco.MjData(m)
        self.joint_map = joint_map
        self.hidden_geoms = set(hidden_geoms)   # 不畫的 geom（示教目標）
        self.timeout = timeout
        self.option = mujoco.MjvOption()
        self.perturb = mujoco.MjvPerturb()
        self.visible = False

    def update(self, msg, d):
        """沒有列出的 joint 與基座沿用 d（目前顯示的姿態）。"""
        self.d.qpos[:] = d.qpos
        self.joint_map.apply(self.d.qpos, msg)
        mujoco.mj_forward(self.m, self.d)
        self.visible = True

    def expired(self, stamp):
        return self.visible and time.monotonic() - stamp > self.timeout

    def draw(self, scn):
        if not self.visible:
            return
        first = scn.ngeom
        mujoco.mjv_addGeoms(self.m, self.d, self.option, self.perturb,
                            mujoco.mjtCatBit.mjCAT_DYNAMIC, scn)
        for i in range(first, scn.ngeom):
            geom = scn.geoms[i]
            if geom.objtype == mujoco.mjtObj.mjOBJ_GEOM and geom.objid in self.hidden_geoms:
                geom.rgba[3] = 0.0
            else:
                geom.rgba[:] = GHOST_RGBA


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=None,
                    help="MJCF path; by default (udp source) ask MujocoSystem which model it loaded")
    ap.add_argument("--source", choices=("udp", "ros"), default="udp",
                    help="udp: MujocoSystem frames (hardware:=mujoco / mujoco_pd); "
                         "ros: --joint-states-topic (hardware:=mock / real / real_pd), needs --model")
    ap.add_argument("--joint-states-topic", default="/joint_states",
                    help="sensor_msgs/JointState shown with --source ros; the teach pose follows its "
                         "joints and order")
    ap.add_argument("--port", type=int, default=9870)
    ap.add_argument("--no-torque", action="store_true", help="hide the joint torque overlay")
    ap.add_argument("--warn-ratio", type=float, default=0.9,
                    help="mark joints whose |torque| / limit is at least this ratio")
    ap.add_argument("--no-arrows", action="store_true", help="hide the joint torque arrows")
    ap.add_argument("--arrow-length", type=float, default=0.3,
                    help="arrow length (m) at the torque limit")
    ap.add_argument("--arrow-scale", type=float, default=0.005,
                    help="arrow length per N*m (m) for actuators without a torque limit")
    ap.add_argument("--arrow-width", type=float, default=0.02, help="arrow shaft width (m)")
    ap.add_argument("--wrist-bodies", default="L_WRIST_R_S,R_WRIST_R_S",
                    help="comma-separated bodies whose axes are drawn (the arm_ik.yaml tip_body)")
    ap.add_argument("--no-wrist-axes", action="store_true", help="hide the wrist axes (Pause toggles)")
    ap.add_argument("--wrist-axis-length", type=float, default=0.12, help="wrist axis length (m)")
    ap.add_argument("--wrist-axis-width", type=float, default=0.008, help="wrist axis width (m)")
    ap.add_argument("--ghost-topic", default="",
                    help="sensor_msgs/JointState drawn as a translucent ghost robot (empty = off), e.g. "
                         "/switch_group_controller/shadow/joint_states")
    ap.add_argument("--ghost-timeout", type=float, default=3.0,
                    help="hide the ghost when nothing was received for this long (s)")
    teach = ap.add_argument_group("teach mode")
    teach.add_argument("--controller", default="/switch_group_controller",
                       help="SwitchGroupController whose ~/playback_state starts / ends teach mode")
    teach.add_argument("--teach-mode", default="teach",
                       help="system mode (controllers.yaml) during which the viewer is in teach mode")
    teach.add_argument("--teach-always", action="store_true",
                       help="stay in teach mode regardless of the controller (e.g. arm_ik_server only)")
    teach.add_argument("--swivel-step", type=float, default=5.0,
                       help="elbow swivel change per F9 / F10 press (deg)")
    teach.add_argument("--arm-ik-server", default="/arm_ik_server",
                       help="graphen_arm_ik arm_ik_server node that solves the IK")
    args = ap.parse_args()
    if args.source == "ros" and args.model is None:
        ap.error("--source ros needs --model (there is no MujocoSystem to ask)")
    if not args.ghost_timeout > 0:
        ap.error("--ghost-timeout must be > 0")

    sock = None
    if args.source == "udp":
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", args.port))
        sock.setblocking(False)

    if args.model is None:
        args.model = query_model(sock, args.port)
        if not Path(args.model).is_file():
            fallback = str(FB70_MJCF / "t170_free.xml")
            print(f"model '{args.model}' reported by MujocoSystem is not on this machine, "
                  f"using {fallback}; pass --model to choose another one")
            args.model = fallback

    m, teach_supported = load_model(args.model)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    frame_bytes = 8 * (1 + m.nq + m.nu)
    labels, limits = actuator_labels(m), actuator_limits(m)
    hinges = actuator_hinges(m)
    joint_map = JointMap(m)
    show_torque = [not args.no_torque]
    show_arrows = [not args.no_arrows]
    show_axes = [not args.no_wrist_axes]
    wrist_bodies = []
    for name in filter(None, (n.strip() for n in args.wrist_bodies.split(","))):
        body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
        if body < 0:
            print(f"wrist axes: body '{name}' is not in the model, skipped")
        else:
            wrist_bodies.append(body)

    ros = None
    try:
        ros = RosLink(args)
    except ImportError as e:
        if args.source == "ros" or args.ghost_topic:
            raise SystemExit(f"{e}; --source ros / --ghost-topic need rclpy: source install/setup.bash "
                             "and run with a python 3.10 that can import rclpy (PY=... bash mujoco_viewer.sh)")
        print(f"ROS disabled ({e}); showing MujocoSystem frames only")

    teacher = None
    teach_disabled = None
    if not teach_supported:
        teach_disabled = f"MuJoCo {mujoco.__version__} at {mujoco.__file__} has no MjSpec (needs >= 3.2)"
    elif ros is None:
        teach_disabled = ("no rclpy: source install/setup.bash and run with a python 3.10 that can "
                          "import rclpy and mujoco 3.11 (PY=... bash mujoco_viewer.sh, see README)")
    else:
        try:
            teacher = Teacher(m, args, ros)
        except (KeyError, ImportError) as e:
            teach_disabled = f"{e}"
    if teach_disabled:
        print(f"teach mode disabled: {teach_disabled}")
    ghost = Ghost(m, joint_map, teacher.geoms if teacher else [], args.ghost_timeout) \
        if ros is not None and args.ghost_topic else None
    # 示教的按鍵由主迴圈處理（key callback 在 viewer thread，不直接改 d）
    teach_keys = queue.SimpleQueue()

    def on_key(key):
        if key == KEY_END:
            if sock is None:
                print("reset is only available with --source udp (MujocoSystem)")
            else:
                sock.sendto(b"reset", ("127.0.0.1", args.port + 1))
                print("reset sent")
        elif key == KEY_HOME:
            show_torque[0] = not show_torque[0]
        elif key == KEY_PAGE_UP:
            show_arrows[0] = not show_arrows[0]
        elif key == KEY_WRIST_AXES:
            show_axes[0] = not show_axes[0]
        elif key in TEACH_KEYS:
            teach_keys.put(key)

    source = (f"listening on 127.0.0.1:{args.port}" if args.source == "udp"
              else f"showing {args.joint_states_topic}")
    teach_trigger = ("always" if args.teach_always
                     else f"while {args.controller} is in mode '{args.teach_mode}'")
    print(f"model {args.model}: nq={m.nq}, nu={m.nu}; {source}; teach mode {teach_trigger} "
          "(End = reset, Home = torque text, PageUp = torque arrows, Pause = wrist axes)")
    base = m.body("BASE_S").id
    last_rx = 0.0
    next_teach_attempt = 0.0
    with mujoco.viewer.launch_passive(m, d, key_callback=on_key) as v:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        v.cam.trackbodyid = base
        v.cam.distance, v.cam.elevation, v.cam.azimuth = 3.0, -15, 135
        arrows_shown = None   # 主迴圈已套用的箭頭狀態；key callback 在 viewer thread，只改旗標
        axes_shown = None
        forces = None         # 最新一筆的致動器力矩（重畫箭頭用）
        overlay = TextOverlay(v)

        def redraw(teaching):
            """重畫 user_scn：力矩箭頭（只在收到的姿態）+ 手腕座標軸 + 幽靈。呼叫前持有 v.lock()."""
            v.user_scn.ngeom = 0
            if arrows_shown and not teaching and forces is not None:
                draw_torque_arrows(v.user_scn, d, hinges, limits, forces, args)
            if axes_shown:
                draw_frame_axes(v.user_scn, d, wrist_bodies, args.wrist_axis_length,
                                args.wrist_axis_width)
            if ghost is not None:
                ghost.draw(v.user_scn)

        def show_torque_text():
            if show_torque[0] and forces is not None:
                names, values = torque_text(labels, limits, forces, args.warn_ratio)
                overlay.show([(mujoco.mjtFontScale.mjFONTSCALE_100,
                               mujoco.mjtGridPos.mjGRID_TOPRIGHT, names, values)])
            else:
                overlay.show([])

        while v.is_running():
            if ros is not None:
                ros.spin()
            teaching = teacher is not None and teacher.active
            if arrows_shown != show_arrows[0] or axes_shown != show_axes[0]:
                arrows_shown, axes_shown = show_arrows[0], show_axes[0]
                with v.lock():
                    v.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = int(arrows_shown)
                    redraw(teaching)
                v.sync()

            # 示教模式跟著 controller 的模式（或 --teach-always）
            if teacher is not None:
                want = args.teach_always or (ros.mode is not None and ros.mode == args.teach_mode)
                if want and not teacher.active and time.monotonic() >= next_teach_attempt:
                    with v.lock():
                        if not teacher.enter(d):
                            next_teach_attempt = time.monotonic() + TEACH_RETRY_PERIOD
                        redraw(teacher.active)
                    v.sync()
                elif not want and teacher.active:
                    with v.lock():
                        teacher.exit()
                        redraw(False)
                    overlay.show([])
                    v.sync()
                teaching = teacher.active
            while not teach_keys.empty():
                key = teach_keys.get()
                if teacher is None:
                    print(f"teach mode disabled: {teach_disabled}")
                elif teacher.active:
                    with v.lock():
                        teacher.handle_key(key)
            # ~/set_pose：示教中直接指定關節角
            if ros is not None and ros.set_pose:
                requests, ros.set_pose[:] = list(ros.set_pose), []
                if teacher is not None and teacher.active:
                    with v.lock():
                        for msg in requests:
                            teacher.apply_joints(d, msg)
                        redraw(True)
                    v.sync()
                else:
                    print(f"/{NODE_NAME}/set_pose ignored: not in teach mode")

            # 收到的姿態：udp 只取最新一筆；ros 為最新的 /joint_states
            latest = None
            if sock is not None:
                try:
                    while True:
                        latest = sock.recv(65536)
                except BlockingIOError:
                    pass
            joint_state = None
            if ros is not None and ros.joint_state is not None:
                joint_state, ros.joint_state = ros.joint_state, None

            # 幽靈：新的姿態或逾時都要重畫
            ghost_changed = False
            if ghost is not None:
                if ros.ghost is not None:
                    msg, ros.ghost = ros.ghost, None
                    with v.lock():
                        ghost.update(msg, d)
                    ghost_changed = True
                elif ghost.expired(ros.ghost_stamp):
                    ghost.visible = False
                    ghost_changed = True

            if teaching:
                # 示教時不套用收到的姿態（不然會蓋掉 IK 的結果）；離開後由下一筆恢復
                if latest is not None or joint_state is not None:
                    last_rx = time.monotonic()
                with v.lock():
                    teacher.update(d, v.perturb)
                    redraw(teaching)
                    names, values = teacher.overlay()
                overlay.show([(mujoco.mjtFontScale.mjFONTSCALE_100,
                               mujoco.mjtGridPos.mjGRID_TOPLEFT, names, values)])
                v.sync()
            elif latest is not None:
                if len(latest) != frame_bytes:
                    print(f"frame size {len(latest)} != {frame_bytes}: model mismatch?")
                else:
                    frame = np.frombuffer(latest, dtype=np.float64)
                    forces = frame[1 + m.nq:]
                    with v.lock():
                        d.time = frame[0]
                        d.qpos[:] = frame[1:1 + m.nq]
                        mujoco.mj_forward(m, d)
                        redraw(teaching)
                    show_torque_text()
                    v.sync()
                    last_rx = time.monotonic()
            elif joint_state is not None:
                with v.lock():
                    if joint_map.apply(d.qpos, joint_state) == 0:
                        print(f"{args.joint_states_topic}: no joint matches the model, model mismatch?")
                    forces = joint_map.forces(joint_state)
                    mujoco.mj_forward(m, d)
                    redraw(teaching)
                show_torque_text()
                v.sync()
                last_rx = time.monotonic()
            elif ghost_changed:
                with v.lock():
                    redraw(teaching)
                v.sync()
            elif time.monotonic() - last_rx > 2.0 and last_rx > 0:
                print(f"no data from {'MujocoSystem' if sock is not None else args.joint_states_topic} "
                      "for 2 s")
                last_rx = 0.0
            time.sleep(0.005)
    if ros is not None:
        ros.close()


if __name__ == "__main__":
    main()
