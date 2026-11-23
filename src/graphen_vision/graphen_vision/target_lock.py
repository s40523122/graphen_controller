"""在多張臉中鎖定一張並持續跟同一張：不依賴 ROS。"""

import math
from typing import List, Optional, Tuple

from graphen_vision.yunet import Face


class TargetLock:
    """每一幀呼叫 update()，回傳鎖定目標平滑後的中心（像素）；這一幀沒有目標時回傳 None。

    - 沒有鎖定時選面積最大的臉（通常是最近的人）。
    - 已鎖定時選中心離上一個位置最近、而且距離在 max_jump（畫面寬度的比例）以內的臉，
      多人時不會因為別人的臉比較大就跳過去。
    - 連續 lost_frames 幀找不到符合的臉才解除鎖定；期間不輸出（BT 以逾時判斷遺失）。
    - 中心以指數平滑：smoothing 為舊值的權重（0 = 不平滑），剛鎖定時直接採用該幀。
    """

    def __init__(self, max_jump: float = 0.25, lost_frames: int = 5, smoothing: float = 0.5):
        if not max_jump > 0.0:
            raise ValueError('max_jump must be greater than zero')
        if lost_frames < 1:
            raise ValueError('lost_frames must be at least 1')
        if not 0.0 <= smoothing < 1.0:
            raise ValueError('smoothing must be in [0, 1)')
        self._max_jump = max_jump
        self._lost_frames = lost_frames
        self._smoothing = smoothing
        self._center: Optional[Tuple[float, float]] = None
        self._missed = 0
        self.locked_face: Optional[Face] = None

    @property
    def locked(self) -> bool:
        return self._center is not None

    def reset(self) -> None:
        self._center = None
        self._missed = 0
        self.locked_face = None

    def update(self, faces: List[Face], image_width: int) -> Optional[Tuple[float, float]]:
        self.locked_face = None
        if self._center is None:
            if not faces:
                return None
            face = max(faces, key=lambda f: f.area)
            self._center = (face.cx, face.cy)
            self._missed = 0
            self.locked_face = face
            return self._center

        max_dist = self._max_jump * image_width
        best = None
        best_dist = math.inf
        for face in faces:
            dist = math.hypot(face.cx - self._center[0], face.cy - self._center[1])
            if dist <= max_dist and dist < best_dist:
                best, best_dist = face, dist
        if best is None:
            self._missed += 1
            if self._missed >= self._lost_frames:
                self.reset()
            return None

        self._missed = 0
        a = self._smoothing
        self._center = (a * self._center[0] + (1.0 - a) * best.cx,
                        a * self._center[1] + (1.0 - a) * best.cy)
        self.locked_face = best
        return self._center
