#!/usr/bin/env python3
"""
IMU 方向性驗證（引導式）.

訂閱 sensor_msgs/Imu，以機器人做得到的小幅動作（直立、前傾、右傾、原地左轉）
檢查加速度、角速度、姿態轉到 base_link 後是否符合 REP-103 FLU（x 前、y 左、z 上）。
只讀取 topic 與 TF，不送任何命令。
"""

import math
import sys
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
import tf2_ros

# 動作步驟開始前先取一小段靜止資料作為起點
REFERENCE_DURATION = 0.5
# 收據時間差超過此值時視為中斷，不納入積分（秒）
MAX_GYRO_DT = 0.1

AXES = ('x', 'y', 'z')


# ---- 向量 / 四元數小工具（四元數順序為 x, y, z, w，與 ROS 訊息一致） ----

def v_add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def v_sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def v_scale(a, s):
    return tuple(x * s for x in a)


def v_dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def v_norm(a):
    return math.sqrt(v_dot(a, a))


def v_mean(vs):
    n = len(vs)
    return tuple(sum(v[i] for v in vs) / n for i in range(3))


def v_angle_deg(a, b):
    na, nb = v_norm(a), v_norm(b)
    if na < 1e-9 or nb < 1e-9:
        return 180.0
    c = max(-1.0, min(1.0, v_dot(a, b) / (na * nb)))
    return math.degrees(math.acos(c))


def v_fmt(a, precision=3):
    return '(' + ', '.join(f'{x:+.{precision}f}' for x in a) + ')'


def dominant_axis(v, min_norm=0.0):
    """回傳絕對值最大的軸與其符號，例如 '-z'；長度不超過 min_norm 時回傳 'none'（動作太小）."""
    if v_norm(v) <= min_norm:
        return 'none'
    i = max(range(3), key=lambda k: abs(v[k]))
    return ('+' if v[i] >= 0 else '-') + AXES[i]


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_normalize(q):
    n = math.sqrt(sum(x * x for x in q))
    return tuple(x / n for x in q) if n > 1e-9 else (0.0, 0.0, 0.0, 1.0)


def q_rotate(q, v):
    """以 q 旋轉向量 v（q ⊗ v ⊗ q*）."""
    p = q_mul(q_mul(q, (v[0], v[1], v[2], 0.0)), q_conj(q))
    return p[:3]


def q_to_rotvec(q):
    """四元數轉旋轉向量（rad），取最短路徑."""
    q = q_normalize(q)
    if q[3] < 0:
        q = tuple(-x for x in q)
    s = v_norm(q[:3])
    if s < 1e-9:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(s, q[3])
    return v_scale(q[:3], angle / s)


def msg_quat(m):
    return (m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w)


def msg_gyro(m):
    return (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)


def msg_accel(m):
    return (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)


class Result:
    """單一檢查項目的結果；observed / expected 用於總結的軸對應."""

    def __init__(self, step, item, passed, detail, expected='', observed=''):
        self.step = step
        self.item = item
        self.passed = passed
        self.detail = detail
        self.expected = expected
        self.observed = observed


class ImuOrientationCheck(Node):

    def __init__(self):
        super().__init__('imu_orientation_check')
        self.declare_parameter('imu_topic', '/imu_sensor_broadcaster/imu')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', '')
        self.declare_parameter('gravity', 9.80665)
        self.declare_parameter('static_duration', 2.0)
        self.declare_parameter('motion_duration', 6.0)
        self.declare_parameter('min_tilt_deg', 5.0)
        self.declare_parameter('min_yaw_deg', 20.0)
        self.declare_parameter('gravity_tolerance', 0.5)
        self.declare_parameter('max_static_gyro', 0.05)
        self.declare_parameter('max_axis_error_deg', 30.0)
        self.declare_parameter('tf_timeout', 5.0)

        p = self.get_parameter
        self.imu_topic = p('imu_topic').value
        self.base_frame = p('base_frame').value
        self.imu_frame = p('imu_frame').value
        self.gravity = float(p('gravity').value)
        self.static_duration = float(p('static_duration').value)
        self.motion_duration = float(p('motion_duration').value)
        self.min_tilt_deg = float(p('min_tilt_deg').value)
        self.min_yaw_deg = float(p('min_yaw_deg').value)
        self.gravity_tolerance = float(p('gravity_tolerance').value)
        self.max_static_gyro = float(p('max_static_gyro').value)
        self.max_axis_error_deg = float(p('max_axis_error_deg').value)
        self.tf_timeout = float(p('tf_timeout').value)

        self._lock = threading.Lock()
        # (接收時間 monotonic, Imu)；broadcaster 的時間戳是 controller_manager 的時間，
        # 積分角速度改用接收時間，避免依賴 use_sim_time 等設定
        self._samples = []
        self._recording = False
        self._last_msg = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Imu, self.imu_topic, self._on_imu, qos_profile_sensor_data)

        self.q_bi = (0.0, 0.0, 0.0, 1.0)
        self.gyro_bias = (0.0, 0.0, 0.0)
        self.results = []

    def _on_imu(self, msg):
        now = time.monotonic()
        with self._lock:
            self._last_msg = msg
            if self._recording:
                self._samples.append((now, msg))

    def last_msg(self):
        with self._lock:
            return self._last_msg

    def record(self, duration):
        with self._lock:
            self._samples = []
            self._recording = True
        time.sleep(duration)
        with self._lock:
            self._recording = False
            return list(self._samples)

    # ---- 準備 ----

    def wait_for_imu(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.last_msg() is not None:
                return True
            time.sleep(0.1)
        return False

    def lookup_mounting(self):
        """查詢 base_frame ← imu_frame 的旋轉，之後所有向量都轉到 base_frame 判斷."""
        imu_frame = self.imu_frame or self.last_msg().header.frame_id
        if not imu_frame or imu_frame == self.base_frame:
            self.get_logger().warn(
                f"IMU frame '{imu_frame}' is empty or equal to base frame; using identity")
            return True
        deadline = time.monotonic() + self.tf_timeout
        while time.monotonic() < deadline:
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, imu_frame, Time())
                r = t.transform.rotation
                self.q_bi = q_normalize((r.x, r.y, r.z, r.w))
                rpy = self._quat_to_rpy_deg(self.q_bi)
                self.get_logger().info(
                    f'Mounting {self.base_frame} <- {imu_frame}: rpy(deg) = {v_fmt(rpy, 1)}')
                return True
            except tf2_ros.TransformException:
                time.sleep(0.1)
        self.get_logger().error(
            f'TF {self.base_frame} <- {imu_frame} not available within {self.tf_timeout} s '
            '(is robot_state_publisher running?)')
        return False

    @staticmethod
    def _quat_to_rpy_deg(q):
        x, y, z, w = q
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return tuple(math.degrees(a) for a in (roll, pitch, yaw))

    def to_base(self, v):
        return q_rotate(self.q_bi, v)

    def add(self, *args, **kwargs):
        r = Result(*args, **kwargs)
        self.results.append(r)
        tag = 'PASS' if r.passed else 'FAIL'
        print(f'  [{tag}] {r.item}: {r.detail}')

    # ---- 步驟 ----

    def step_upright(self):
        step = 'upright'
        samples = self.record(self.static_duration)
        if len(samples) < 2:
            print('  [FAIL] no IMU data received during the step')
            self.results.append(Result(step, 'data', False, 'no data'))
            return
        msgs = [m for _, m in samples]
        a_b = self.to_base(v_mean([msg_accel(m) for m in msgs]))
        gyro_imu = v_mean([msg_gyro(m) for m in msgs])
        self.gyro_bias = gyro_imu
        g_b = self.to_base(gyro_imu)

        # 加速度計量到的是比力：靜止時朝上，大小約 g
        norm_err = abs(v_norm(a_b) - self.gravity)
        tilt = v_angle_deg(a_b, (0.0, 0.0, 1.0))
        self.add(step, 'accel', norm_err < self.gravity_tolerance and tilt < self.min_tilt_deg,
                 f'a = {v_fmt(a_b)} m/s^2, ||a| - g| = {norm_err:.3f}, '
                 f'angle to +z = {tilt:.1f} deg',
                 expected='+z', observed=dominant_axis(a_b))

        self.add(step, 'gyro (static)', v_norm(g_b) < self.max_static_gyro,
                 f'mean w = {v_fmt(g_b, 4)} rad/s (kept as bias)')

        # 由姿態推算 IMU 座標系中的「上」方向，與加速度方向比對（yaw 不可觀測，只驗證 roll / pitch）
        q_wi = q_normalize(msg_quat(msgs[-1]))
        up_b = self.to_base(q_rotate(q_conj(q_wi), (0.0, 0.0, 1.0)))
        err = v_angle_deg(up_b, a_b)
        self.add(step, 'orientation', err < self.min_tilt_deg,
                 f'up from quaternion = {v_fmt(up_b)}, angle to accel = {err:.1f} deg',
                 expected='+z', observed=dominant_axis(up_b))
        if msgs[-1].orientation_covariance[0] == -1.0:
            print('  note: orientation_covariance[0] is -1 (broadcaster marks orientation as '
                  'unavailable); quaternion values are checked anyway')

    def step_motion(self, step, action, axis, min_angle_deg, expect_tilt):
        """
        動作步驟：先記錄起點，再記錄動作過程.

        axis 是預期的旋轉軸（base_frame，FLU）；expect_tilt 為 True 時同時檢查加速度
        的變化方向（傾斜時重力在機身座標系中朝旋轉的反方向偏移）。
        """
        print('  hold still...')
        ref = self.record(REFERENCE_DURATION)
        print(f'  GO: {action} now, then hold until done ({self.motion_duration:.0f} s)')
        motion = self.record(self.motion_duration)
        print('  done, return the robot to upright')
        if len(ref) < 2 or len(motion) < 2:
            print('  [FAIL] no IMU data received during the step')
            self.results.append(Result(step, 'data', False, 'no data'))
            return

        e = tuple(1.0 if AXES[i] == axis else 0.0 for i in range(3))
        expected = '+' + axis
        a0 = self.to_base(v_mean([msg_accel(m) for _, m in ref]))
        q0 = q_normalize(msg_quat(ref[-1][1]))
        # 結束時取最後一段平均，降低晃動的影響
        t_end = motion[-1][0]
        tail = [m for t, m in motion if t_end - t <= REFERENCE_DURATION]
        a1 = self.to_base(v_mean([msg_accel(m) for m in tail]))
        q1 = q_normalize(msg_quat(motion[-1][1]))

        # 角速度扣除零偏後積分（IMU 座標系），再轉到 base
        theta = (0.0, 0.0, 0.0)
        prev_t = ref[-1][0]
        for t, m in motion:
            dt = t - prev_t
            prev_t = t
            if 0.0 < dt <= MAX_GYRO_DT:
                theta = v_add(theta, v_scale(v_sub(msg_gyro(m), self.gyro_bias), dt))
        theta_b = self.to_base(theta)
        theta_deg = v_scale(theta_b, 180.0 / math.pi)

        # 加速度
        if expect_tilt:
            # 繞 +e 正向旋轉 → 比力（朝上）在機身座標系中朝 -(e × ẑ) = (-e_y, e_x, 0) 偏移
            # 前傾（+y）：ax < 0；右傾（+x）：ay > 0
            shift_dir = (-e[1], e[0], 0.0)
            delta = v_sub(a1, a0)
            component = v_dot(a1, shift_dir)
            threshold = self.gravity * math.sin(math.radians(self.min_tilt_deg))
            self.add(step, 'accel', component > threshold,
                     f'a_end = {v_fmt(a1)} m/s^2, component along {dominant_axis(shift_dir)} = '
                     f'{component:+.3f} (need > {threshold:.3f})',
                     expected=dominant_axis(shift_dir), observed=dominant_axis(delta, threshold))
        else:
            change = v_angle_deg(a0, a1)
            self.add(step, 'accel', change < self.min_tilt_deg,
                     f'gravity direction changed {change:.1f} deg (need < {self.min_tilt_deg})')

        # 角速度
        along = v_dot(theta_deg, e)
        gyro_ok = dominant_axis(theta_b) == expected and along > min_angle_deg
        self.add(step, 'gyro', gyro_ok,
                 f'integrated = {v_fmt(theta_deg, 1)} deg, along {expected} = {along:+.1f} '
                 f'(need > {min_angle_deg})',
                 expected=expected, observed=dominant_axis(theta_deg, min_angle_deg))

        # 姿態：機身座標系中的姿態變化 q0⁻¹ ⊗ q1，以預期軸判定（不依賴陀螺，避免陀螺錯誤時連帶判錯）；
        # 與積分角速度的夾角只列出供參考
        rot_deg = v_scale(self.to_base(q_to_rotvec(q_mul(q_conj(q0), q1))), 180.0 / math.pi)
        err = v_angle_deg(rot_deg, e)
        orient_ok = v_dot(rot_deg, e) > min_angle_deg and err < self.max_axis_error_deg
        self.add(step, 'orientation', orient_ok,
                 f'rotation = {v_fmt(rot_deg, 1)} deg, angle to {expected} = {err:.1f} deg '
                 f'(need < {self.max_axis_error_deg}), '
                 f'angle to integrated gyro = {v_angle_deg(rot_deg, theta_deg):.1f} deg',
                 expected=expected, observed=dominant_axis(rot_deg, min_angle_deg))

    # ---- 總結 ----

    def summarize(self):
        print('\n===== Summary =====')
        for r in self.results:
            mapping = f'  expected {r.expected}, observed {r.observed}' if r.expected else ''
            print(f"  {'PASS' if r.passed else 'FAIL'}  {r.step:<12} {r.item:<14}{mapping}")

        if all(r.passed for r in self.results):
            print('\nAll checks passed: output matches REP-103 FLU in '
                  f'{self.base_frame}.')
            return True

        # 動作太小（observed 為 none）不能用來判斷軸方向，另外提示
        too_small = [r for r in self.results if not r.passed and r.observed == 'none']

        def failed(item):
            return [r for r in self.results
                    if r.item == item and not r.passed and r.observed != 'none']

        accel_bad, gyro_bad, orient_bad = failed('accel'), failed('gyro'), failed('orientation')
        print('\nHints:')
        if too_small:
            steps = sorted({r.step for r in too_small})
            print(f"  - motion too small in: {', '.join(steps)}; re-run with a larger motion.")
        if accel_bad:
            print('  - accel axis/sign mismatch: check axis_transform (imu16460 hardware) '
                  'or imu_rpy (URDF mounting).')
        if gyro_bad and not accel_bad:
            print('  - accel OK but gyro mismatch: gyro sign convention differs from the '
                  'accelerometer; needs a separate fix in the driver.')
        if orient_bad and not (accel_bad or gyro_bad):
            print('  - accel and gyro OK but orientation mismatch: check euler_order and the '
                  'pitch/yaw sign assumptions in Imu16460SensorInterface::write_sample().')
        if failed('gyro (static)'):
            print('  - gyro not near zero while static: robot was moving or gyro bias is large.')
        if failed('data'):
            print(f'  - no IMU data during some steps: check {self.imu_topic}.')
        print('  - a motion in the wrong direction also fails the checks; re-run if unsure.')
        return False


def prompt(text):
    """回傳 False 表示使用者選擇跳過這一步."""
    answer = input(f'\n>> {text}\n   Press Enter to start (s = skip): ')
    return answer.strip().lower() != 's'


def run(node):
    print(f'Waiting for IMU messages on {node.imu_topic} ...')
    if not node.wait_for_imu(5.0):
        node.get_logger().error(f'No messages on {node.imu_topic} within 5 s')
        return False
    if not node.lookup_mounting():
        return False

    print('\nIMU orientation check (expected frame: REP-103 FLU, x forward / y left / z up).')
    print('Keep motions small (10-15 deg) and make sure the robot is secured (e.g. on a gantry).')

    if prompt('[1/4] Robot upright and still.'):
        node.step_upright()
    steps = [
        ('lean_forward', '[2/4] Robot upright; after GO, lean it FORWARD ~10-15 deg.',
         'lean forward', 'y', node.min_tilt_deg, True),
        ('lean_right', '[3/4] Robot upright; after GO, lean it to its RIGHT ~10-15 deg.',
         'lean right', 'x', node.min_tilt_deg, True),
        ('turn_left', '[4/4] Robot upright; after GO, turn it LEFT in place '
         '(counter-clockwise seen from above) ~30-90 deg.',
         'turn left', 'z', node.min_yaw_deg, False),
    ]
    for step, text, action, axis, min_angle, tilt in steps:
        if prompt(text):
            node.step_motion(step, action, axis, min_angle, tilt)
    return node.summarize()


def main():
    # 操作提示與 ROS log 交錯輸出，改為逐行 flush 以維持順序
    sys.stdout.reconfigure(line_buffering=True)
    rclpy.init()
    node = ImuOrientationCheck()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass  # 主 thread 呼叫 rclpy.try_shutdown() 結束時的正常路徑

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()
    ok = False
    try:
        ok = run(node)
    except (KeyboardInterrupt, EOFError):
        print('\nAborted.')
    finally:
        # 先讓 spin 結束並 join，再銷毀 node，否則結束時可能 abort
        rclpy.try_shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
