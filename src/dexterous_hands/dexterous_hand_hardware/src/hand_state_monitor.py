#!/usr/bin/env python3
import rclpy
import time
import math
from enum import Enum
from dataclasses import dataclass
from collections import deque
import json
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from control_msgs.msg import DynamicJointState
from std_msgs.msg import String, Int32


# ============================================================
# 1. 狀態列舉（Enum）
# ============================================================
class JointStatus(Enum):
    RELEASING = 0
    GRASPING = 1
    POSITION_REACHED = 2
    FORCE_REACHED = 3
    CURRENT_PROTECTION = 5
    STALL_PROTECTION = 6
    FAULT = 7

class GraspResult(Enum):
    EMPTY = "抓空"
    HOLDING_OBJECT = "抓到物體"
    UNKNOWN = "未知"

class SlipState(Enum):
    STABLE = "穩定"
    WARNING = "滑動警告"
    SLIPPING = "正在滑動"

class HandOverallState(Enum):
    IDLE = "待機"
    RELEASING = "鬆開中"
    GRASPING = "抓取中"
    HOLDING_STABLE = "穩定持握"
    SLIP_ALERT = "滑動警報"
    DISTURBANCE_ALERT = "外部拉扯"   # ← 新增
    SAFETY_FAULT = "安全異常"
    


# ============================================================
# 2. 資料容器（Dataclass）
# ============================================================
@dataclass
class JointData:
    position: float
    velocity: float
    effort: float
    current: float
    error: float
    status: int
    temperature: float

@dataclass
class TactileData:
    normal: float
    tangent: float
    direction: float


# ============================================================
# 3. 單指偵測器
# ============================================================
class FingerDetector:
    """
    單一手指的感測狀態偵測器。
    負責三件事:
      1. 判斷是否接觸(is_contact)
      2. 判斷抓取結果(grasp_result) —— 空氣 / 抓到物體 / 未知
      3. 判斷滑動狀態(detect_slip) —— 持握中的物體是否正在滑走
      4. 判斷外部干擾(detect_disturbance) —— 已穩定持握時,是否被外力(如拉扯)介入
    """

    def __init__(self, name,
                 contact_th=0.3,
                 friction_limit=0.5,
                 direction_drift_th=0.8,
                 temp_limit=60.0,
                 stabilize_duration=0.3,
                 direction_window=6,
                 debounce_frames=5,
                 exit_ratio=0.5,
                 min_tangent_for_direction=0.05,
                 grasp_effort_th=None,
                 jerk_th=0.5,
                 velocity_anomaly_th=0.02,
                 disturbance_cooldown=0.5):
        self.name = name

        # --- 接觸判斷 ---
        self.contact_th = contact_th

        # --- 滑動判斷 ---
        self.friction_limit = friction_limit
        self.direction_enter_th = direction_drift_th
        self.direction_exit_th = direction_drift_th * exit_ratio
        self.min_tangent_for_direction = min_tangent_for_direction
        self.direction_window = direction_window
        self.debounce_frames = debounce_frames

        # --- 安全門檻 ---
        self.temp_limit = temp_limit

        # --- 穩定基準線建立 ---
        self.stabilize_duration = stabilize_duration

        # --- 抓取判斷 ---
        self.grasp_effort_th = grasp_effort_th

        # --- 外部干擾判斷 ---
        self.jerk_th = jerk_th
        self.velocity_anomaly_th = velocity_anomaly_th
        self.disturbance_cooldown = disturbance_cooldown

        # --- 內部狀態:滑動偵測用 ---
        self.tangent_history = deque(maxlen=10)
        self.direction_history = deque(maxlen=direction_window)

        # --- 內部狀態:基準線與接觸追蹤 ---
        self.baseline = None
        self.was_contact = False
        self.contact_start_ts = None
        self.baseline_samples = []
        self.is_stable = False

        # --- 內部狀態:滑動防抖計數器 ---
        self.slip_streak = 0
        self.stable_streak = 0
        self.warn_streak = 0
        self.current_slip_state = SlipState.STABLE

        # --- 內部狀態:外部干擾偵測用(完全獨立,不與 update_baseline 共用) ---
        self._prev_tactile = None
        self._prev_ts = None
        self._prev_valid = False      # 上一幀是否處於「穩定接觸中」
        self._was_above_th = False
        self._last_disturbance_ts = None

    # =========================================================
    # 1. 接觸判斷
    # =========================================================
    def is_contact(self, tactile: TactileData) -> bool:
        return tactile.normal > self.contact_th

    # =========================================================
    # 2. 基準線維護(每一幀都要呼叫,且必須在 detect_slip 之前)
    #    注意:這裡已不再動 disturbance 相關的內部狀態,
    #    避免「接觸剛消失」的那一幀,把拉扯訊號的證據提早清空。
    # =========================================================
    def update_baseline(self, tactile: TactileData, contact: bool, now: float):
        if contact and not self.was_contact:
            self.contact_start_ts = now
            self.baseline_samples = [tactile]
            self.baseline = None
            self.is_stable = False
            self.tangent_history.clear()
            self.direction_history.clear()
            self._reset_slip_tracking()

        elif contact and self.was_contact:
            if not self.is_stable:
                self.baseline_samples.append(tactile)
                elapsed = now - self.contact_start_ts
                if elapsed >= self.stabilize_duration:
                    self.baseline = self._average_tactile(self.baseline_samples)
                    self.is_stable = True
                    self.baseline_samples = []
                    self._reset_slip_tracking()

        else:
            self.baseline = None
            self.baseline_samples = []
            self.contact_start_ts = None
            self.is_stable = False
            self.direction_history.clear()
            self._reset_slip_tracking()

        self.was_contact = contact

    def _reset_slip_tracking(self):
        self.slip_streak = 0
        self.stable_streak = 0
        self.warn_streak = 0
        self.current_slip_state = SlipState.STABLE

    @staticmethod
    def _average_tactile(samples) -> "TactileData":
        n = len(samples)
        avg_normal = sum(s.normal for s in samples) / n
        avg_tangent = sum(s.tangent for s in samples) / n
        sin_sum = sum(math.sin(s.direction) for s in samples)
        cos_sum = sum(math.cos(s.direction) for s in samples)
        avg_direction = math.atan2(sin_sum, cos_sum)
        return TactileData(normal=avg_normal, tangent=avg_tangent, direction=avg_direction)

    @staticmethod
    def _circular_mean(angles) -> float:
        sin_sum = sum(math.sin(a) for a in angles)
        cos_sum = sum(math.cos(a) for a in angles)
        return math.atan2(sin_sum, cos_sum)

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        d = (a - b) % (2 * math.pi)
        if d > math.pi:
            d -= 2 * math.pi
        return abs(d)

    # =========================================================
    # 3. 抓取結果判斷
    # =========================================================
    def grasp_result(self, joint: JointData, contact: bool, slip: SlipState) -> GraspResult:
        try:
            status = JointStatus(joint.status)
        except ValueError:
            return GraspResult.UNKNOWN

        if status == JointStatus.POSITION_REACHED:
            return GraspResult.EMPTY

        if status in (JointStatus.FORCE_REACHED, JointStatus.CURRENT_PROTECTION):
            if self.grasp_effort_th is not None:
                return GraspResult.HOLDING_OBJECT if joint.effort > self.grasp_effort_th \
                    else GraspResult.EMPTY
            if contact and slip in (SlipState.WARNING, SlipState.SLIPPING):
                return GraspResult.HOLDING_OBJECT
            return GraspResult.UNKNOWN

        return GraspResult.UNKNOWN

    # =========================================================
    # 4. 滑動偵測(需在已穩定持握,即 is_stable=True 時才有效)
    # =========================================================
    def detect_slip(self, tactile: TactileData, contact: bool) -> SlipState:
        if not contact or not self.is_stable or self.baseline is None:
            self._reset_slip_tracking()
            return SlipState.STABLE

        delta_tangent = tactile.tangent - self.baseline.tangent
        ratio = delta_tangent / max(tactile.normal, 1e-3)
        slipping_by_force = ratio > self.friction_limit

        slipping_by_direction = False
        min_tangent = self.min_tangent_for_direction
        if tactile.tangent > min_tangent and self.baseline.tangent > min_tangent:
            self.direction_history.append(tactile.direction)
            if len(self.direction_history) == self.direction_history.maxlen:
                half = len(self.direction_history) // 2
                history_list = list(self.direction_history)
                older_avg = self._circular_mean(history_list[:half])
                newer_avg = self._circular_mean(history_list[half:])
                drift = self._angle_diff(newer_avg, older_avg)
                active_th = self.direction_exit_th if self.current_slip_state == SlipState.SLIPPING \
                    else self.direction_enter_th
                slipping_by_direction = drift > active_th

        raw_slip = slipping_by_force or slipping_by_direction

        if raw_slip:
            self.stable_streak = 0
            self.slip_streak += 1
            self.warn_streak = 0
        elif ratio > self.friction_limit * 0.7:
            self.stable_streak = 0
            self.slip_streak = 0
            self.warn_streak += 1
        else:
            self.stable_streak += 1
            self.slip_streak = 0
            self.warn_streak = 0

        if self.slip_streak >= self.debounce_frames:
            self.current_slip_state = SlipState.SLIPPING
        elif self.warn_streak >= self.debounce_frames // 2:
            self.current_slip_state = SlipState.WARNING
        elif self.stable_streak >= self.debounce_frames:
            self.current_slip_state = SlipState.STABLE

        return self.current_slip_state

    # =========================================================
    # 5. 外部干擾偵測(例如:有人拉扯已持握的物體)
    #
    # 設計理念:
    #   - 用自己獨立管理的 _prev_tactile / _prev_ts / _prev_valid,
    #     完全不依賴 update_baseline,避免「接觸剛消失」的那一幀
    #     被提早清空,導致拉扯瞬間的訊號永遠算不出來。
    #   - 即使這一幀 contact 已經變成 False(輕物被整個拉走),
    #     只要「上一幀」是穩定接觸的,就會用上一幀 vs 這一幀的落差
    #     算出變化速度 —— 這正是拉扯發生的關鍵瞬間。
    # =========================================================
    def detect_disturbance(self, tactile: TactileData, joint: JointData,
                            contact: bool, now: float) -> dict | None:
        prev_tactile = self._prev_tactile
        prev_ts = self._prev_ts
        prev_valid = self._prev_valid

        currently_valid = contact and self.is_stable
        event = None

        if prev_valid and prev_tactile is not None and prev_ts is not None:
            dt = now - prev_ts
            if dt > 0:
                d_tangent = tactile.tangent - prev_tactile.tangent
                d_normal = tactile.normal - prev_tactile.normal
                rate = math.sqrt(d_tangent ** 2 + d_normal ** 2) / dt
                is_above_th = rate > self.jerk_th

                if is_above_th and not self._was_above_th:
                    if self._last_disturbance_ts is None or \
                            (now - self._last_disturbance_ts) > self.disturbance_cooldown:
                        self._last_disturbance_ts = now
                        event = {"finger": self.name, "onset_ts": now, "rate": rate}
                self._was_above_th = is_above_th

        if currently_valid:
            self._prev_tactile = tactile
            self._prev_ts = now
        else:
            self._prev_tactile = None
            self._prev_ts = None
            self._was_above_th = False

        self._prev_valid = currently_valid
        return event

    # =========================================================
    # 6. 安全檢查(硬體異常警示,與抓取/滑動判斷無關)
    # =========================================================
    def safety_check(self, joint: JointData) -> list:
        alerts = []
        try:
            status = JointStatus(joint.status)
        except ValueError:
            return alerts
        if status == JointStatus.STALL_PROTECTION:
            alerts.append(f"{self.name}: 關節堵轉")
        if status == JointStatus.FAULT:
            alerts.append(f"{self.name}: 硬體故障")
        if joint.temperature > self.temp_limit:
            alerts.append(f"{self.name}: 溫度過高 ({joint.temperature}°C)")
        return alerts



# ============================================================
# 3.5 每根手指的客製化參數
#     左右手若是同型號硬體,兩組可以填一樣的數值;
#     若硬體不同,再分開調整即可。
# ============================================================
FINGER_CONFIGS_RIGHT = {
    "thumb":  dict(contact_th=0.35, friction_limit=0.55, direction_drift_th=0.8, grasp_effort_th=-1.10),
    "index":  dict(contact_th=0.30, friction_limit=0.50, direction_drift_th=0.8, grasp_effort_th=2.0),
    "middle": dict(contact_th=0.30, friction_limit=0.50, direction_drift_th=0.8),
    "ring":   dict(contact_th=0.30, friction_limit=0.50, direction_drift_th=0.8),
    "pinky":  dict(contact_th=0.20, friction_limit=0.40, direction_drift_th=0.9),
}

FINGER_CONFIGS_LEFT = {
    "thumb":  dict(contact_th=0.35, friction_limit=0.55, direction_drift_th=0.8),
    "index":  dict(contact_th=0.30, friction_limit=0.50, direction_drift_th=0.8),
    "middle": dict(contact_th=0.30, friction_limit=0.50, direction_drift_th=0.8),
    "ring":   dict(contact_th=0.30, friction_limit=0.50, direction_drift_th=0.8),
    "pinky":  dict(contact_th=0.20, friction_limit=0.40, direction_drift_th=0.9),
}


# ============================================================
# 4. 整手狀態機
# ============================================================
class HandStateDetector:
    def __init__(self, finger_configs: dict = None,
                 disturbance_sync_window: float = 0.15,
                 disturbance_min_fingers: int = 1):
        finger_configs = finger_configs or {}
        self.fingers = {
            name: FingerDetector(name, **finger_configs.get(name, {}))
            for name in ["thumb", "index", "middle", "ring", "pinky"]
        }
        self.palm_zones = ["palm_left", "palm_middle", "palm_right"]

        self.disturbance_sync_window = disturbance_sync_window
        self.disturbance_min_fingers = disturbance_min_fingers
        self._pending_disturbances = {}

    def evaluate(self, joint_data: dict, tactile_data: dict, now: float) -> dict:
        report = {
            "fingers": {}, "palm_contact": {}, "alerts": [],
            "overall": None, "disturbance": False,
            "disturbance_fingers": [], "disturbance_onset_ts": None,
        }
        slip_flags = []
        grasp_flags = []
        disturbance_hits = []
        moving_statuses = []

        for name, detector in self.fingers.items():
            j = joint_data.get(name)
            t = tactile_data.get(name)
            if j is None or t is None:
                continue

            contact = detector.is_contact(t)
            detector.update_baseline(t, contact, now)

            slip = detector.detect_slip(t, contact)
            grasp = detector.grasp_result(j, contact, slip)

            alerts = detector.safety_check(j)

            disturbance_event = detector.detect_disturbance(t, j, contact, now)
            if disturbance_event:
                disturbance_hits.append(disturbance_event)

            report["fingers"][name] = {
                "contact": contact, "grasp": grasp, "slip": slip,
                "disturbance": disturbance_event is not None,
            }
            report["alerts"].extend(alerts)

            if grasp == GraspResult.HOLDING_OBJECT:
                grasp_flags.append(True)
            if slip in (SlipState.WARNING, SlipState.SLIPPING):
                slip_flags.append(slip)

            try:
                moving_statuses.append(JointStatus(j.status))
            except ValueError:
                pass

        # ---- 多指同步驗證(外部拉扯) ----
        for ev in disturbance_hits:
            self._pending_disturbances[ev["finger"]] = ev["onset_ts"]

        self._pending_disturbances = {
            k: v for k, v in self._pending_disturbances.items()
            if now - v <= self.disturbance_sync_window
        }

        confirmed_disturbance = len(self._pending_disturbances) >= self.disturbance_min_fingers
        if confirmed_disturbance:
            onset_ts = min(self._pending_disturbances.values())
            fingers_involved = list(self._pending_disturbances.keys())
            report["disturbance"] = True
            report["disturbance_onset_ts"] = onset_ts
            report["disturbance_fingers"] = fingers_involved
            report["alerts"].append(
                f"⚠️ 偵測到外部拉扯,起始時間:{onset_ts:.3f},涉及手指:{fingers_involved}"
            )
            self._pending_disturbances.clear()

        for zone in self.palm_zones:
            t = tactile_data.get(zone)
            if t:
                report["palm_contact"][zone] = t.normal > 0.5

        any_grasping = JointStatus.GRASPING in moving_statuses
        any_releasing = JointStatus.RELEASING in moving_statuses

        hardware_alerts = [a for a in report["alerts"] if "⚠️" not in a]

        if hardware_alerts:
            report["overall"] = HandOverallState.SAFETY_FAULT
        elif confirmed_disturbance:
            report["overall"] = HandOverallState.DISTURBANCE_ALERT
        elif SlipState.SLIPPING in slip_flags:
            report["overall"] = HandOverallState.SLIP_ALERT
        elif any_grasping:
            report["overall"] = HandOverallState.GRASPING
        elif any_releasing:
            report["overall"] = HandOverallState.RELEASING
        elif any(grasp_flags):
            report["overall"] = HandOverallState.HOLDING_STABLE
        else:
            report["overall"] = HandOverallState.IDLE

        return report



# ============================================================
# 5. ROS2 Node：讀取資料 + 呼叫偵測邏輯
# ============================================================
FINGER_KEYWORDS = ['thumb', 'index', 'middle', 'ring', 'pinky']
REQUIRED_FINGERS = {'thumb', 'index', 'middle', 'ring', 'pinky'}

# ← 這裡調整偵測頻率，數字越小頻率越高
# 100Hz = 0.01, 200Hz = 0.005，建議依實際硬體發布速度調整，
# 設得比硬體更新速度還快沒有意義，只是白白耗 CPU
DETECT_PERIOD_SEC = 0.1


class HandReader(Node):
    def __init__(self, side: str = 'right', joint_states_topic: str = None):
        """
        side: 'left' 或 'right',決定這個節點負責哪隻手
              - 決定訂閱關節名稱的前綴（left_ / right_）
              - 決定套用哪一組 FINGER_CONFIGS
              - 決定發布狀態的 Topic 名稱
        joint_states_topic: 如果左右手是分開的 Topic 才需要指定,
                             若左右手共用同一個 /dynamic_joint_states,留空即可
        """
        assert side in ('left', 'right'), "side 只能是 'left' 或 'right'"

        node_name = f'{side}_hand_reader'
        super().__init__(node_name)

        self.side = side
        self.prefix = f'{side}_'   # 用來過濾關節名稱，例如 'right_' 或 'left_'

        finger_configs = FINGER_CONFIGS_RIGHT if side == 'right' else FINGER_CONFIGS_LEFT
        self.detector = HandStateDetector(finger_configs=finger_configs)
        self.last_snapshot = None

        # 暫存最新一筆解析後的資料，callback 跟 timer 共用
        self.latest_joint_data = {}
        self.latest_tactile_data = {}
        self.has_data = False

        topic = joint_states_topic or '/dynamic_joint_states'
        self.create_subscription(
            DynamicJointState,
            topic,
            self.callback,
            10
        )

        # 高頻計時器：偵測頻率由這裡決定，不受訊息到達速度限制
        self.create_timer(DETECT_PERIOD_SEC, self.detect_loop)

        state_topic = f'/{side}_hand/overall_state'
        self.state_pub = self.create_publisher(String, state_topic, 10)

        finger_state_topic = f'/{side}_hand/finger_state'
        self.finger_state_pub = self.create_publisher(Int32, finger_state_topic, 10)

        self.get_logger().info(
            f'[{node_name}] 已啟動！訂閱 {topic}（前綴: {self.prefix}），發布至 {state_topic}'
        )

    # ---------- 訊息回調：只做解析，越輕量越好 ----------
    def callback(self, msg):
        joint_data, tactile_data = self.parse_message(msg)
        if joint_data:
            self.latest_joint_data = joint_data
        if tactile_data:
            self.latest_tactile_data = tactile_data
        self.has_data = True

        # 安全事件仍維持「一收到訊息就立即檢查」，不等 timer
        self._immediate_safety_check(joint_data)

    def parse_message(self, msg):
        joint_data = {}
        tactile_data = {}

        for i, name in enumerate(msg.joint_names):
            if not name.startswith(self.prefix):   # ← 依 side 動態決定前綴，不再寫死 'right_'
                continue

            iface = msg.interface_values[i]
            values = dict(zip(iface.interface_names, iface.values))

            if 'tactile_normal' in values:
                key = self._extract_key(name)
                tactile_data[key] = TactileData(
                    normal=values.get('tactile_normal', 0.0),
                    tangent=values.get('tactile_tangent', 0.0),
                    direction=values.get('tactile_direction', 0.0),
                )
            elif 'position' in values:
                key = self._extract_key(name)
                joint_data[key] = JointData(
                    position=values.get('position', 0.0),
                    velocity=values.get('velocity', 0.0),
                    effort=values.get('effort', 0.0),
                    current=values.get('current', 0.0),
                    error=values.get('error', 0.0),
                    status=int(values.get('status', -1)),
                    temperature=values.get('temperature', 0.0),
                )

        return joint_data, tactile_data

    def _extract_key(self, name: str) -> str:
        for finger in FINGER_KEYWORDS:
            if finger in name:
                return finger
        if 'palm' in name:
            suffix = name.split('palm')[-1]
            if 'right' in suffix:
                return 'palm_right'
            if 'left' in suffix:
                return 'palm_left'
            if 'middle' in suffix:
                return 'palm_middle'
        return name

    # ---------- 高頻偵測迴圈：真正做判斷的地方 ----------
    def detect_loop(self):
        if not self.has_data:
            return
        if not REQUIRED_FINGERS.issubset(self.latest_joint_data.keys()):
            return
        if not REQUIRED_FINGERS.issubset(self.latest_tactile_data.keys()):
            return

        now = time.time()
        report = self.detector.evaluate(self.latest_joint_data, self.latest_tactile_data, now)
        snapshot = self._build_snapshot(report)

        if snapshot != self.last_snapshot:
            self.last_snapshot = snapshot
            self._print_full_report(report)

        state_dict = self._build_state_dict(report)
        msg = String()
        msg.data = json.dumps(state_dict, ensure_ascii=False)
        self.state_pub.publish(msg)

    def _build_snapshot(self, report: dict) -> tuple:
        finger_snapshot = tuple(
            (name, info["contact"], info["grasp"], info["slip"])
            for name, info in sorted(report["fingers"].items())
        )
        palm_snapshot = tuple(sorted(report["palm_contact"].items()))
        alerts_snapshot = tuple(sorted(report["alerts"]))
        return (report["overall"], finger_snapshot, palm_snapshot, alerts_snapshot)

    def _build_state_dict(self, report: dict) -> dict:
        """把 report 轉成純字串/布林值組成的字典,才能被 json.dumps() 序列化"""
        fingers_dict = {}
        for name, info in report["fingers"].items():
            fingers_dict[name] = {
                "contact": info["contact"],
                "grasp": info["grasp"].name,
                "slip": info["slip"].name,
                "disturbance": info["disturbance"],
            }

        return {
            "side": self.side,
            "overall": report["overall"].name,
            "fingers": fingers_dict,
            "palm_contact": report["palm_contact"],
            "alerts": report["alerts"],
            "disturbance": report["disturbance"],
            "disturbance_fingers": report["disturbance_fingers"],
            "disturbance_onset_ts": report["disturbance_onset_ts"],
        }

    # ---------- 顯示全部手指狀態 ----------
    def _print_full_report(self, report: dict):
        print(f"\n{'='*50}")
        print(f"[{self.side.upper()} 狀態變化] {time.strftime('%H:%M:%S')}  整手狀態: {report['overall'].value}")
        print(f"{'-'*50}")
        for index, name in enumerate(["thumb", "index", "middle", "ring", "pinky"]):
            info = report["fingers"].get(name)
            if info:
                if info["disturbance"]:
                    # publish 滑動訊號
                    finger_idx = Int32()
                    finger_idx.data = index
                    self.finger_state_pub.publish(finger_idx)
                    
                    mark = " ⚡"  
                else:
                    mark = ""
                print(f"  {name:8s} | 接觸:{str(info['contact']):5s} | "
                    f"抓取:{info['grasp'].value:6s} | 滑動:{info['slip'].value}{mark}")
        print(f"{'-'*50}")
        for zone, touched in report["palm_contact"].items():
            print(f"  {zone:12s} | 接觸:{touched}")
        if report["disturbance"]:
            print(f"  外部拉扯確認! 起始時間:{report['disturbance_onset_ts']:.3f} "
                f"涉及手指:{report['disturbance_fingers']}")
        if report["alerts"]:
            print(f"  警告: {report['alerts']}")
        print(f"{'='*50}")


    def _immediate_safety_check(self, joint_data: dict):
        for name, j in joint_data.items():
            try:
                status = JointStatus(j.status)
            except ValueError:
                continue
            if status in (JointStatus.STALL_PROTECTION, JointStatus.FAULT):
                self.get_logger().error(f'🚨 [{self.side}] 緊急: {name} 發生 {status.name}')


def main(args=None):
    rclpy.init(args=args)

    right_reader = HandReader(side='right')
    # left_reader = HandReader(side='left')

    executor = MultiThreadedExecutor()
    executor.add_node(right_reader)
    # executor.add_node(left_reader)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        right_reader.destroy_node()
        # left_reader.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
