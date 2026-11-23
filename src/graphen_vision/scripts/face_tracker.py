#!/usr/bin/env python3
"""人臉跟隨的視覺端：相機影像 → 鎖定一張臉 → 發布人臉中心的像素（給 BT 的 face_track.xml）。

影像來源（參數 source）：
  topic   訂閱 sensor_msgs/Image（例如 realsense2_camera 發布的 color 影像）
  device  直接讀取 V4L2 裝置（webcam，或不安裝 RealSense SDK、把它的彩色串流當 webcam），
          在背景 thread 讀取與處理，不需要任何相機驅動節點
以 YuNet 偵測，鎖定一張臉後發布 ~/target（geometry_msgs/PointStamped，point.x / .y 為原始影像的像素；
topic 模式沿用影像的 header，device 模式為讀到該幀的時間與 frame_id 參數）。
沒有鎖定的目標時不發布：BT 以逾時判斷遺失。
"""

import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from graphen_vision.device_capture import DeviceCapture
from graphen_vision.target_lock import TargetLock
from graphen_vision.yunet import YuNetDetector
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

_DEFAULT_MODEL = 'face_detection_yunet_2023mar.onnx'


class FaceTracker(Node):

    def __init__(self):
        super().__init__('face_tracker')
        p = self.declare_parameter
        self._source = p('source', 'topic').value
        image_topic = p('image_topic', '/camera/camera/color/image_raw').value
        device = str(p('device', '/dev/video0').value)
        device_width = p('device_width', 640).value
        device_height = p('device_height', 480).value
        device_fps = p('device_fps', 30.0).value
        device_fourcc = p('device_fourcc', 'YUYV').value
        self._frame_id = p('frame_id', 'camera_color_optical_frame').value
        self._reopen_after = p('device_reopen_after', 2.0).value
        if self._source not in ('topic', 'device'):
            raise ValueError(f"source must be 'topic' or 'device', got '{self._source}'")
        model_path = p('model_path', '').value
        detect_width = p('detect_width', 320).value
        score_threshold = p('score_threshold', 0.6).value
        nms_threshold = p('nms_threshold', 0.3).value
        top_k = p('top_k', 50).value
        min_face_px = p('min_face_px', 20.0).value
        lock_max_jump = p('lock_max_jump', 0.25).value
        lock_lost_frames = p('lock_lost_frames', 5).value
        smoothing = p('smoothing', 0.5).value
        self._expected_width = p('expected_width', 0).value
        self._expected_height = p('expected_height', 0).value
        self._publish_debug = p('publish_debug_image', False).value
        self._stats_period = p('stats_period', 5.0).value
        image_reliable = p('image_reliable', True).value
        self._no_image_warn = p('no_image_warn_period', 5.0).value

        if not model_path:
            model_path = os.path.join(
                get_package_share_directory('graphen_vision'), 'models', _DEFAULT_MODEL)
        # 參數不合法時建構失敗（main 印出原因後結束）
        self._detector = YuNetDetector(model_path, detect_width, score_threshold, nms_threshold,
                                       top_k, min_face_px)
        self._lock = TargetLock(lock_max_jump, lock_lost_frames, smoothing)
        self._bridge = CvBridge()

        self._target_pub = self.create_publisher(PointStamped, '~/target', 10)
        self._debug_pub = (self.create_publisher(Image, '~/debug_image', QoSProfile(depth=1))
                           if self._publish_debug else None)
        self._size_logged = False
        self._size_mismatch_warned = False
        self._frames = 0
        self._published = 0
        self._detect_ms = 0.0
        self._stats_start = time.monotonic()
        self._last_image = time.monotonic()

        self._stop = threading.Event()
        self._capture = None
        self._thread = None
        if self._source == 'topic':
            # 佇列深度 1：處理不及時只保留最新的一幀，延遲不會累積。
            # reliability 要和發送端相容：best effort 在大影像（約 1 MB）時，分片遺失會讓大部分的幀收不到
            # （實測跨 process 只收到約 1 成），realsense2_camera 預設以 reliable 發布
            qos = QoSProfile(
                depth=1, history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE,
                reliability=(ReliabilityPolicy.RELIABLE if image_reliable
                             else ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(Image, image_topic, self._on_image, qos)
            input_name = image_topic
        else:
            self._capture = DeviceCapture(device, device_width, device_height, device_fps,
                                          device_fourcc)
            # 打不開時丟出 RuntimeError（main 印出原因後結束）
            width, height, fps = self._capture.open()
            self.get_logger().info(f'Opened {device}: {width}x{height} @ {fps:.0f} fps '
                                   f'(requested {device_width}x{device_height} {device_fourcc})')
            self._thread = threading.Thread(target=self._device_loop, daemon=True)
            self._thread.start()
            input_name = device
        if self._no_image_warn > 0.0:
            self.create_timer(self._no_image_warn, self._check_images)
        self.get_logger().info(
            f'Face tracker on [ {input_name} ] ({self._source}) with {os.path.basename(model_path)} '
            f'(detect width {detect_width}), publishing {self._target_pub.topic_name}')

    def destroy_node(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._capture is not None:
            self._capture.close()
        return super().destroy_node()

    def _device_loop(self):
        # read() 會阻塞到下一幀，所以在自己的 thread 讀取與處理；publisher 可以跨 thread 使用
        failing_since = None
        while not self._stop.is_set():
            image = self._capture.read()
            if image is None:
                now = time.monotonic()
                failing_since = failing_since or now
                if now - failing_since >= self._reopen_after:
                    # 拔插或驅動重置後，重新開啟通常就能恢復
                    self.get_logger().warn('Video device stopped delivering frames; reopening',
                                           throttle_duration_sec=10.0)
                    try:
                        self._capture.open()
                    except RuntimeError as e:
                        self.get_logger().warn(str(e), throttle_duration_sec=10.0)
                    failing_since = now
                time.sleep(0.05)
                continue
            failing_since = None
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self._frame_id
            self._process(image, header)

    def _check_images(self):
        # topic 名稱或 QoS 不相容（例如發送端是 best effort 而這裡要求 reliable）時不會有任何錯誤，只是收不到
        if time.monotonic() - self._last_image > self._no_image_warn:
            hint = ('check image_topic and image_reliable (ros2 topic info -v <topic>)'
                    if self._source == 'topic' else
                    'check device (v4l2-ctl --list-devices; RealSense: the YUYV node is the color one)')
            self.get_logger().warn(f'No image for {self._no_image_warn:.0f} s: {hint}',
                                   throttle_duration_sec=30.0)

    def _on_image(self, msg: Image):
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:  # noqa: BLE001 — 編碼不支援時只略過這一幀
            self._last_image = time.monotonic()
            self.get_logger().warn(f'Cannot convert image ({msg.encoding}): {e}',
                                   throttle_duration_sec=5.0)
            return
        self._process(image, msg.header)

    def _process(self, image, header: Header):
        self._last_image = time.monotonic()
        height, width = image.shape[:2]
        if not self._check_size(width, height):
            return

        start = time.monotonic()
        faces = self._detector.detect(image)
        self._detect_ms += (time.monotonic() - start) * 1000.0
        center = self._lock.update(faces, width)
        self._frames += 1

        if center is not None:
            target = PointStamped()
            target.header = header
            target.point.x, target.point.y = center
            self._target_pub.publish(target)
            self._published += 1

        if self._debug_pub is not None:
            self._publish_debug_image(image, faces, center, header)
        self._log_stats()

    def _check_size(self, width: int, height: int) -> bool:
        if not self._size_logged:
            self._size_logged = True
            self.get_logger().info(
                f'Image size {width}x{height}: gp_face_image_width / gp_face_image_height '
                'of bt_runner must match')
        expected_w, expected_h = self._expected_width, self._expected_height
        if (expected_w > 0 and width != expected_w) or (expected_h > 0 and height != expected_h):
            # 尺寸和 BT 的設定不一致時，誤差的比例與中心都會錯：不發布，讓 BT 視為遺失
            if not self._size_mismatch_warned:
                self._size_mismatch_warned = True
                self.get_logger().warn(
                    f'Image size {width}x{height} differs from the expected '
                    f'{expected_w}x{expected_h}; not publishing targets')
            return False
        return True

    def _publish_debug_image(self, image, faces, center, header):
        for face in faces:
            color = (0, 255, 0) if face is self._lock.locked_face else (0, 160, 255)
            cv2.rectangle(image, (int(face.x), int(face.y)),
                          (int(face.x + face.w), int(face.y + face.h)), color, 2)
            cv2.putText(image, f'{face.score:.2f}', (int(face.x), int(face.y) - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        if center is not None:
            cv2.drawMarker(image, (int(center[0]), int(center[1])), (0, 0, 255),
                           cv2.MARKER_CROSS, 20, 2)
        h, w = image.shape[:2]
        cv2.drawMarker(image, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_TILTED_CROSS, 12, 1)
        debug = self._bridge.cv2_to_imgmsg(image, encoding='bgr8')
        debug.header = header
        self._debug_pub.publish(debug)

    def _log_stats(self):
        if self._stats_period <= 0.0:
            return
        elapsed = time.monotonic() - self._stats_start
        if elapsed < self._stats_period:
            return
        self.get_logger().info(
            f'{self._frames / elapsed:.1f} fps, detect {self._detect_ms / max(self._frames, 1):.1f} ms, '
            f'published {self._published}, locked: {self._lock.locked}')
        self._frames = 0
        self._published = 0
        self._detect_ms = 0.0
        self._stats_start = time.monotonic()


def main():
    rclpy.init()
    node = None
    try:
        node = FaceTracker()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except (ValueError, RuntimeError, cv2.error) as e:
        # 參數或模型有誤：印出原因後結束
        rclpy.logging.get_logger('face_tracker').fatal(f'Face tracker failed: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
