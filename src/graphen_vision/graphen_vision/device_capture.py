"""直接讀取 V4L2 影像裝置（webcam，或把 RealSense 的彩色串流當 webcam）：不依賴 ROS。

RealSense D4xx 是 UVC 裝置，不安裝 librealsense 也能以 Linux 的 uvcvideo 讀取彩色影像；
它會出現好幾個 /dev/video*（深度、紅外線、彩色、metadata），要指定彩色那一個（格式 YUYV）。
編號在插拔後可能改變，建議用 /dev/v4l/by-id/... 的路徑。
也可以指定影片檔（測試用）：播完時 read() 回傳 None，由呼叫端重新開啟。
"""

import time
from typing import Optional, Tuple

import cv2
import numpy as np


class DeviceCapture:

    def __init__(self, device: str, width: int = 0, height: int = 0, fps: float = 0.0,
                 fourcc: str = ''):
        if fourcc and len(fourcc) != 4:
            raise ValueError(f"device_fourcc must be 4 characters (e.g. YUYV, MJPG) or empty, got '{fourcc}'")
        # 純數字視為裝置編號，其他視為路徑
        self._device = int(device) if device.isdigit() else device
        self._width = int(width)
        self._height = int(height)
        self._fps = float(fps)
        self._fourcc = fourcc
        self._cap: Optional[cv2.VideoCapture] = None
        # 影片檔依自身的 fps 播放（裝置本身就以相機的速度送出，不需要）
        self._frame_period = 0.0
        self._next_frame = 0.0

    def open(self) -> Tuple[int, int, float]:
        """開啟裝置並套用格式；回傳實際的 (寬, 高, fps)。打不開時丟出 RuntimeError。"""
        self.close()
        # 裝置用 V4L2；其他路徑（影片檔，測試用）交給 OpenCV 自動選擇後端
        is_device = isinstance(self._device, int) or str(self._device).startswith('/dev/')
        cap = cv2.VideoCapture(self._device, cv2.CAP_V4L2 if is_device else cv2.CAP_ANY)
        if not cap.isOpened():
            raise RuntimeError(f'cannot open video device {self._device}')
        # 先設定格式再設定解析度：有些裝置的解析度清單依格式而不同
        if self._fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self._fourcc))
        if self._width > 0:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        if self._height > 0:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        if self._fps > 0.0:
            cap.set(cv2.CAP_PROP_FPS, self._fps)
        # 驅動端只保留最新的一幀，延遲不會累積（部分驅動不支援，忽略結果）
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._cap = cap
        file_fps = cap.get(cv2.CAP_PROP_FPS) if not is_device else 0.0
        self._frame_period = 1.0 / file_fps if file_fps > 0.0 else 0.0
        self._next_frame = time.monotonic()
        return (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                float(cap.get(cv2.CAP_PROP_FPS)))

    def read(self) -> Optional[np.ndarray]:
        """讀取一幀（BGR）；失敗時回傳 None。會阻塞到下一幀到達。"""
        if self._cap is None:
            return None
        if self._frame_period > 0.0:
            delay = self._next_frame - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            self._next_frame = max(self._next_frame + self._frame_period, time.monotonic())
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return None
        if frame.ndim == 2:
            # 紅外線等單通道串流：轉成三通道讓偵測可以執行（通常表示指定了錯誤的裝置）
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        return frame

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

