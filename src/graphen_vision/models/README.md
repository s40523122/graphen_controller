# 模型

| 檔案 | 來源 | sha256 | 授權 |
|---|---|---|---|
| `face_detection_yunet_2023mar.onnx` | opencv_zoo（https://huggingface.co/opencv/face_detection_yunet） | `8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4` | MIT（`LICENSE_yunet`） |

- 2021dec 版（OpenCV 4.5.4 的 `cv2.FaceDetectorYN` 能直接使用的格式）已從 opencv_zoo 下架。
  2023mar 版需要 OpenCV ≥ 4.8 的 `FaceDetectorYN`，Humble 的系統 OpenCV 是 4.5.4，
  所以 `graphen_vision/yunet.py` 以 `cv2.dnn` 執行並自行解碼。
- 輸入的寬高會補到 32 的倍數（模型內部的上取樣要求）。
