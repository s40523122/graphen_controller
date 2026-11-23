"""YuNet 人臉偵測（cv2.dnn）：不依賴 ROS。

模型為 opencv_zoo 的 face_detection_yunet_2023mar.onnx。OpenCV 4.8 以後的 cv2.FaceDetectorYN
可以直接使用這個模型，但 ROS 2 Humble 的系統 OpenCV 是 4.5.4（只支援已下架的 2021dec 格式），
所以這裡以 cv2.dnn 執行，後處理（解碼、NMS）依 OpenCV 4.8 的 FaceDetectorYN 實作。
"""

from dataclasses import dataclass
import math
import os
from typing import List

import cv2
import numpy as np

# 模型的三個輸出尺度；輸入的寬高必須是最大 stride 的倍數
_STRIDES = (8, 16, 32)
_INPUT_ALIGN = 32


@dataclass
class Face:
    """原始影像座標（像素）中的一張臉。"""

    x: float
    y: float
    w: float
    h: float
    score: float

    @property
    def cx(self) -> float:
        return self.x + self.w / 2.0

    @property
    def cy(self) -> float:
        return self.y + self.h / 2.0

    @property
    def area(self) -> float:
        return self.w * self.h


class YuNetDetector:
    """以 detect_width 縮小影像後偵測，回傳原始影像座標的人臉框。"""

    def __init__(self, model_path: str, detect_width: int = 320,
                 score_threshold: float = 0.6, nms_threshold: float = 0.3,
                 top_k: int = 50, min_face_px: float = 0.0):
        if detect_width < _INPUT_ALIGN:
            raise ValueError(f'detect_width must be at least {_INPUT_ALIGN}, got {detect_width}')
        if not 0.0 < score_threshold < 1.0:
            raise ValueError('score_threshold must be in (0, 1)')
        if not 0.0 < nms_threshold < 1.0:
            raise ValueError('nms_threshold must be in (0, 1)')
        if top_k < 1:
            raise ValueError('top_k must be at least 1')
        # OpenCV 對缺檔只回報 "Can't read ONNX file"：先檢查，指出最常見的原因
        if not os.path.isfile(model_path):
            raise ValueError(
                f'YuNet model not found: {model_path} (it ships in graphen_vision/models/; '
                'make sure the .onnx file was deployed and rebuild graphen_vision)')
        self._net = cv2.dnn.readNet(model_path)
        self._output_names = self._net.getUnconnectedOutLayersNames()
        self._detect_width = int(detect_width)
        self._score_threshold = float(score_threshold)
        self._nms_threshold = float(nms_threshold)
        self._top_k = int(top_k)
        self._min_face_px = float(min_face_px)

    def detect(self, image_bgr: np.ndarray) -> List[Face]:
        height, width = image_bgr.shape[:2]
        # 縮到 detect_width（不放大），保持長寬比；之後把框乘回原尺寸
        scale = min(1.0, self._detect_width / float(width))
        small_w = max(1, int(round(width * scale)))
        small_h = max(1, int(round(height * scale)))
        small = image_bgr if scale == 1.0 else cv2.resize(
            image_bgr, (small_w, small_h), interpolation=cv2.INTER_AREA)

        # 右下補 0 到 32 的倍數（模型內部的上取樣要求各尺度的大小一致）
        pad_w = int(math.ceil(small_w / _INPUT_ALIGN) * _INPUT_ALIGN)
        pad_h = int(math.ceil(small_h / _INPUT_ALIGN) * _INPUT_ALIGN)
        padded = cv2.copyMakeBorder(small, 0, pad_h - small_h, 0, pad_w - small_w,
                                    cv2.BORDER_CONSTANT, value=(0, 0, 0))

        # 與 FaceDetectorYN 相同：BGR、不縮放數值、不減平均
        self._net.setInput(cv2.dnn.blobFromImage(padded))
        outputs = dict(zip(self._output_names, self._net.forward(self._output_names)))

        boxes, scores = self._decode(outputs, pad_w, pad_h)
        if not scores:
            return []
        keep = cv2.dnn.NMSBoxes(boxes, scores, self._score_threshold, self._nms_threshold,
                                top_k=self._top_k)
        faces = []
        for i in np.array(keep).reshape(-1):
            x, y, w, h = boxes[i]
            face = Face(x / scale, y / scale, w / scale, h / scale, scores[i])
            if min(face.w, face.h) >= self._min_face_px:
                faces.append(face)
        return faces

    def _decode(self, outputs, pad_w: int, pad_h: int):
        boxes = []
        scores = []
        for stride in _STRIDES:
            cols = pad_w // stride
            rows = pad_h // stride
            cls = outputs[f'cls_{stride}'].reshape(-1)
            obj = outputs[f'obj_{stride}'].reshape(-1)
            bbox = outputs[f'bbox_{stride}'].reshape(-1, 4)
            if cls.size != rows * cols:
                raise RuntimeError(
                    f'unexpected YuNet output size for stride {stride}: {cls.size} != {rows * cols}')
            score = np.sqrt(np.clip(cls, 0.0, 1.0) * np.clip(obj, 0.0, 1.0))
            for idx in np.nonzero(score >= self._score_threshold)[0]:
                r, c = divmod(int(idx), cols)
                cx = (c + bbox[idx, 0]) * stride
                cy = (r + bbox[idx, 1]) * stride
                w = math.exp(bbox[idx, 2]) * stride
                h = math.exp(bbox[idx, 3]) * stride
                boxes.append([float(cx - w / 2.0), float(cy - h / 2.0), float(w), float(h)])
                scores.append(float(score[idx]))
        return boxes, scores
