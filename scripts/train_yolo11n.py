#!/usr/bin/env python3
"""
Train YOLO11n-seg model on the full Car-Silhouettes-Forest dataset for SAGE TerraScout.
Dataset classes: Apache friendly, Ka-52 Enemy
Results are saved to: runs_yolo/train/yolo11n_trained/
"""

import sys
from pathlib import Path
from ultralytics import YOLO

def main():
    root = Path(__file__).resolve().parent.parent
    data_yaml = root / "dataset" / "DataSetFull.v2i (2).yolov8" / "data.yaml"
    project_dir = root / "runs_yolo" / "train"

    print(f"Loading data YAML: {data_yaml}")
    if not data_yaml.exists():
        sys.exit(f"Error: Full dataset not found at {data_yaml}")

    # Load YOLO11n-seg pretrained segmentation baseline
    model = YOLO("yolo11n-seg.pt")

    # Train segmentation model with identical augmentations and early stopping
    results = model.train(
        data=str(data_yaml),
        epochs=100,
        imgsz=640,
        batch=16,
        device=0,
        workers=4,
        project=str(project_dir),
        name="yolo11n_trained",
        exist_ok=True,
        plots=True,
        save=True,
        patience=20,
        verbose=True,

        # Augmentations (identical to YOLOv8 for fair comparison):
        degrees=10.0,       # Small rotation angles: -10° to +10°
        hsv_v=0.15,         # Brightness / Exposure: ±15%
        hsv_s=0.15,         # Saturation / Color tone: ±15%
        scale=0.5,          # Scale variations
        perspective=0.0005, # Mild perspective
        fliplr=0.5,         # 50% horizontal flip
        mosaic=1.0,         # Mosaic augmentation
    )

    best_pt = project_dir / "yolo11n_trained" / "weights" / "best.pt"
    print(f"\n[DONE] YOLO11 Training complete! Best weights saved to: {best_pt}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
