# YOLOv8n-seg Trained Model Weights

This directory contains the trained instance segmentation weights for the SAGE TerraScout 2D perception system.

- **File**: `best.pt`
- **Architecture**: YOLOv8n-seg (nano segmentation, 85 layers, 3.26M parameters, 11.3 GFLOPs)
- **Input Resolution**: 640x640 px
- **Weight Size**: ~6.5 MB (optimizer stripped for git tracking)
- **Dataset**: `Car-Silhouettes-Forest.v1i.yolov8`
- **Target Classes**:
  0. `Apache friendly`
  1. `Ka-52 Enemy`

## Performance Metrics (Trained with EarlyStopping, Best at Epoch 66 on NVIDIA GTX 1650 Ti)

### 1. Bounding Box Detection Metrics

| Class | Images | Instances | Precision (P) | Recall (R) | mAP@50 | mAP@50-95 |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **All Classes** | **33** | **35** | **99.5%** | **100.0%** | **99.5%** | **89.4%** |
| `Apache friendly` | 17 | 17 | **99.8%** | **100.0%** | **99.5%** | **87.2%** |
| `Ka-52 Enemy` | 18 | 18 | **99.2%** | **100.0%** | **99.5%** | **91.7%** |

### 2. Instance Segmentation (Mask) Metrics

| Class | Images | Instances | Precision (P) | Recall (R) | mAP@50 | mAP@50-95 |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **All Classes** | **33** | **35** | **96.7%** | **97.2%** | **99.0%** | **78.2%** |
| `Apache friendly` | 17 | 17 | **99.8%** | **100.0%** | **99.5%** | **82.0%** |
| `Ka-52 Enemy` | 18 | 18 | **93.6%** | **94.4%** | **98.5%** | **74.4%** |

- **Inference Latency**: ~10.2 ms per frame (GTX 1650 Ti GPU)
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
