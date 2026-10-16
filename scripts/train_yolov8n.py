#!/usr/bin/env python3
"""
Train YOLOv8n model on the traffic signs and cones dataset for SAGE TerraScout.
Dataset classes: pedestrian_sign, roadwork_sign, stop-sign, traffic-cone
"""

import sys
from pathlib import Path
from ultralytics import YOLO

def main():
    root = Path(__file__).resolve().parent.parent
    data_yaml = root / "dataset" / "cone-traffic-data.v1i.yolov8" / "data.yaml"
    project_dir = root / "runs_yolo" / "train"

    print(f"Loading data YAML: {data_yaml}")
    if not data_yaml.exists():
        sys.exit(f"Data config not found: {data_yaml}")

    # Load YOLOv8n pretrained baseline
    model = YOLO("yolov8n.pt")

    # Train with augmentations for rotation (tilted signs), scaling (far signs), and robustness
    results = model.train(
        data=str(data_yaml),
        epochs=25,
        imgsz=640,
        batch=16,
        device=0,
        workers=4,
        project=str(project_dir),
        name="yolov8n_trained",
        exist_ok=True,
        plots=True,
        save=True,
        patience=8,
        verbose=True,
        # Augmentations to fix real-world issues:
        degrees=25.0,       # Random rotation ±25° (fixes tilted pedestrian & roadwork signs)
        scale=0.6,          # Scale jitter 0.4x - 1.6x (forces model to detect distant small signs)
        perspective=0.0005, # Mild perspective transformation
        fliplr=0.5,         # 50% horizontal flip
        mosaic=1.0,         # Mosaic augmentation (combines 4 scenes into 1, great for small signs)
    )

    best_pt = project_dir / "yolov8n_trained" / "weights" / "best.pt"
    print(f"\n[DONE] Training complete! Best weights saved to: {best_pt}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
