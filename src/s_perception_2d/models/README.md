# YOLOv8n Trained Model Weights

This directory contains the trained weights for the SAGE TerraScout 2D perception system.

- **File**: `best.pt`
- **Architecture**: YOLOv8n (nano, ~3.0M parameters)
- **Input Resolution**: 640x640 px
- **Weight Size**: ~6.2 MB (optimizer stripped for git tracking)
- **Target Classes**:
  0. `pedestrian_sign`
  1. `roadwork_sign`
  2. `stop-sign`
  3. `traffic-cone`

## Final Performance Metrics (25 Epochs on NVIDIA GTX 1650 Ti)

| Class | Images | Instances | Precision (P) | Recall (R) | mAP@50 | mAP@50-95 |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **All Classes** | **509** | **492** | **95.6%** | **94.1%** | **96.1%** | **72.0%** |
| `roadwork_sign` | 32 | 32 | **100.0%** | **99.8%** | **99.5%** | 86.1% |
| `stop-sign` | 187 | 188 | **99.5%** | **98.8%** | **99.5%** | 80.5% |
| `pedestrian_sign` | 53 | 70 | **96.3%** | **92.9%** | **96.4%** | 70.1% |
| `traffic-cone` | 122 | 202 | **86.8%** | **84.8%** | **89.0%** | 51.2% |

- **Inference Latency**: ~5.6 ms per frame (GPU) / ~55 ms (CPU)
- **Model File Size**: 6.0 MB (safe for Git push)

## Loading
The weights are automatically located by `s_perception_2d` when launched:
```bash
ros2 launch s_perception_2d yolo_detector.launch.py
```
Or loaded in Python:
```python
from ultralytics import YOLO
model = YOLO("best.pt")
```
