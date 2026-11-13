# YOLOv8n-seg Trained Model Weights

This directory contains the trained instance segmentation weights for the SAGE TerraScout 2D perception system.

- **File**: `best.pt`
- **Architecture**: YOLOv8n-seg (nano segmentation, 85 layers, 3.26M parameters, 11.3 GFLOPs)
- **Input Resolution**: 640x640 px
- **Weight Size**: ~6.5 MB (optimizer stripped for git tracking)
- **Dataset**: `DataSetFull.v2i (2).yolov8` (313 total images: 219 train, 62 val, 32 test)
- **Target Classes**:
  0. `Apache friendly`
  1. `Ka-52 Enemy`

## Performance Metrics (Trained on Full Dataset with EarlyStopping on NVIDIA GTX 1650 Ti)

### 1. Bounding Box Detection Metrics

| Class | Images | Instances | Precision (P) | Recall (R) | mAP@50 | mAP@50-95 |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **All Classes** | **62** | **78** | **97.9%** | **91.0%** | **92.3%** | **82.3%** |
| `Apache friendly` | 37 | 37 | **100.0%** | **99.1%** | **99.5%** | **88.4%** |
| `Ka-52 Enemy` | 37 | 41 | **95.8%** | **82.9%** | **85.1%** | **76.3%** |

### 2. Instance Segmentation (Mask) Metrics

| Class | Images | Instances | Precision (P) | Recall (R) | mAP@50 | mAP@50-95 |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **All Classes** | **62** | **78** | **96.5%** | **89.8%** | **90.1%** | **67.6%** |
| `Apache friendly` | 37 | 37 | **100.0%** | **99.1%** | **99.5%** | **78.2%** |
| `Ka-52 Enemy` | 37 | 41 | **93.0%** | **80.5%** | **80.7%** | **56.9%** |

- **Inference Latency**: ~8.5 ms per frame (GTX 1650 Ti GPU)
- **Display Output**: The ROS 2 detector node (`s_perception_2d`) renders standard 2D bounding boxes, class labels, and $(u, v)$ center coordinates on `/oakd/detections_image` (mask rendering is suppressed).

## Three-Tier Confidence Filtering Logic
1. **$\text{Confidence} \ge 0.50$**: Confirmed identification (e.g. `Apache friendly`, `Ka-52 Enemy`).
2. **$0.15 < \text{Confidence} < 0.50$**: Uncertain / low confidence $\rightarrow$ Label overridden to `unknown` and published.
3. **$\text{Confidence} \le 0.15$**: Ignored as background noise.

## Loading
The weights are automatically loaded by `s_perception_2d` when launched:
```bash
ros2 launch s_perception_2d yolo_detector.launch.py
```
Or in Python:
```python
from ultralytics import YOLO
model = YOLO("best.pt")
```
