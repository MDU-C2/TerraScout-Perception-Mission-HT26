#!/usr/bin/env python3
"""
Demonstration script showing the complete 2D perception pipeline step-by-step:
1. RGB Image Input
2. YOLOv8n Inference
3. Detection + class + confidence + 2D bounding box
4. Perception validation and ObjectRecord handling
5. Configured class vs unknown classification
6. ROS 2 Detection2D conversion & JSONL logging
"""

import sys
import os
import glob
from pathlib import Path
import cv2
import torch
from ultralytics import YOLO

# Add package root to sys.path
_pkg_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_pkg_root))

from s_perception_2d.object_record import ObjectRecord, ObjectRecordHandler
from s_perception_2d.perception_logger import PerceptionLogger


def print_step(title):
    print("\n" + "=" * 70)
    print(f" {title} ")
    print("=" * 70)


def main():
    print("\n" + "🚀 STARTING PERCEPTION PIPELINE DEMONSTRATION 🚀".center(70))
    
    # -------------------------------------------------------------
    # STEP 1: RGB Image Input
    # -------------------------------------------------------------
    print_step("STEP 1: RGB Camera Image Input")
    test_img_dir = Path("/home/sadra/sage-0.1.0-extracted/dataset/combined_dataset.v1i.yolov8/test/images")
    img_files = list(test_img_dir.glob("*.jpg"))
    if not img_files:
        print("No test images found!")
        return

    # Choose a sample image known to contain a target object
    sample_img_path = img_files[0]
    # Try finding an image with warning_bag or stop-sign for clear demo
    for p in img_files:
        if "warning" in p.name.lower() or "stop" in p.name.lower() or "000035" in p.name or "112" in p.name:
            sample_img_path = p
            break

    cv_image = cv2.imread(str(sample_img_path))
    h, w, c = cv_image.shape
    print(f"📷 Loaded Image: {sample_img_path.name}")
    print(f"   Image Resolution: {w} x {h} (Width x Height), Channels: {c}")

    # -------------------------------------------------------------
    # STEP 2: YOLOv8n Inference
    # -------------------------------------------------------------
    print_step("STEP 2: YOLOv8n Model Inference")
    model_path = _pkg_root / "models" / "best.pt"
    device = 0 if torch.cuda.is_available() else "cpu"
    print(f"🧠 Loading Model: {model_path} (6.0 MB)")
    print(f"⚡ Device: {device} ({torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU'})")
    
    model = YOLO(str(model_path))
    print(f"🏷️  Model Trained Classes: {model.names}")
    
    results = model.predict(source=cv_image, conf=0.35, iou=0.45, imgsz=640, device=device, verbose=False)
    boxes = results[0].boxes
    print(f"🔍 Raw detections found: {len(boxes)}")

    # -------------------------------------------------------------
    # STEP 3 & 4: Detection Validation & ObjectRecord Handling
    # -------------------------------------------------------------
    print_step("STEP 3 & 4: Validation & ObjectRecord Handling")
    configured_classes = ["pedestrian_sign", "roadwork_sign", "stop-sign", "warning_bag"]
    handler = ObjectRecordHandler(
        configured_classes=configured_classes,
        confidence_threshold=0.35,
        allow_unknown_records=True
    )
    print(f"📋 Configured Supported Classes: {configured_classes}")
    print(f"🛡️  Unknown Fallback Allowed: {handler.allow_unknown_records}")

    for idx, box in enumerate(boxes, 1):
        xyxy = box.xyxy[0].cpu().numpy().tolist()
        conf = float(box.conf[0].item())
        cls_id = int(box.cls[0].item())
        raw_name = model.names.get(cls_id, str(cls_id))

        print(f"\n--- Detection #{idx} ---")
        print(f"   Raw YOLO Output:")
        print(f"     Class ID: {cls_id} ('{raw_name}')")
        print(f"     Confidence: {conf:.4f} ({conf*100:.1f}%)")
        print(f"     Bounding Box [x1, y1, x2, y2]: [{xyxy[0]:.1f}, {xyxy[1]:.1f}, {xyxy[2]:.1f}, {xyxy[3]:.1f}]")

        # Create & validate ObjectRecord
        record = handler.validate_and_create_record(
            raw_class_name=raw_name,
            confidence=conf,
            xyxy=(xyxy[0], xyxy[1], xyxy[2], xyxy[3]),
            frame_id="oakd_rgb_camera_optical_frame",
            timestamp=1791542000.0,
            img_width=w,
            img_height=h
        )

        is_configured = (record.class_name in configured_classes)
        print(f"   Perception-System Validation:")
        print(f"     Is Valid Record: {record.is_valid}")
        print(f"     Class Assignment: '{record.class_name}'" + (" (Configured class)" if is_configured else " (Mapped to unknown)"))
        print(f"     Center (u, v): ({record.center_u:.2f}, {record.center_v:.2f})")
        print(f"     Dimensions: Width={record.width:.2f}px, Height={record.height:.2f}px")

    # -------------------------------------------------------------
    # STEP 5: Demonstration of 'unknown' Fallback Handling
    # -------------------------------------------------------------
    print_step("STEP 5: Demonstration of 'unknown' Class Handling")
    print("Testing what happens when an unsupported object or unlisted class is detected:")
    unsupported_record = handler.validate_and_create_record(
        raw_class_name="traffic_cone",  # Not in configured classes
        confidence=0.88,
        xyxy=(50.0, 60.0, 150.0, 200.0),
        frame_id="oakd_rgb_camera_optical_frame",
        timestamp=1791542001.0,
        img_width=w,
        img_height=h
    )
    print(f"   Input Raw Class: 'traffic_cone' (Confidence: 0.88)")
    print(f"   Output ObjectRecord class_name: '{unsupported_record.class_name}'")
    print(f"   Is in Configured Classes: {unsupported_record.class_name in configured_classes}")
    print(f"   Center (u, v): ({unsupported_record.center_u:.1f}, {unsupported_record.center_v:.1f})")

    # -------------------------------------------------------------
    # STEP 6: JSONL Validation Logging Output
    # -------------------------------------------------------------
    print_step("STEP 6: Structured JSONL Logging Format")
    logger = PerceptionLogger(log_dir="/tmp/demo_perception_logs")
    logger.log_frame_detections(
        frame_id="oakd_rgb_camera_optical_frame",
        timestamp=1791542000.0,
        records=[record, unsupported_record],
        image_shape=(h, w)
    )
    with open(logger.jsonl_path, "r") as f:
        print("Logged Record in JSONL:")
        for line in f:
            print(f"   {line.strip()}")

    print("\n" + "✅ PERCEPTION PIPELINE DEMONSTRATION COMPLETE ✅".center(70) + "\n")


if __name__ == "__main__":
    main()
