#!/usr/bin/env python3
"""
Test script: publishes a real test image with a warning_bag,
listens for /oakd/detections_2d and /oakd/detections_image,
and saves the annotated output frame for visual verification.
"""

import time
import sys
from pathlib import Path
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge


class PerceptionTester(Node):
    def __init__(self, test_img_path: str):
        super().__init__("perception_tester")
        self.bridge = CvBridge()
        self.received_detection = None
        self.received_annotated_img = None

        self.pub_cam = self.create_publisher(Image, "/test_oakd/rgb/image_raw", 10)
        self.sub_det = self.create_subscription(
            Detection2DArray, "/oakd/detections_2d", self.det_cb, 10
        )
        self.sub_annot = self.create_subscription(
            Image, "/oakd/detections_image", self.annot_cb, 10
        )

        self.cv_img = cv2.imread(test_img_path)
        if self.cv_img is None:
            raise FileNotFoundError(f"Cannot load image: {test_img_path}")

    def det_cb(self, msg: Detection2DArray):
        if len(msg.detections) > 0:
            self.received_detection = msg

    def annot_cb(self, msg: Image):
        self.received_annotated_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def run_test(self, timeout_sec=5.0):
        img_msg = self.bridge.cv2_to_imgmsg(self.cv_img, encoding="bgr8")
        img_msg.header.frame_id = "oakd_rgb_camera_optical_frame"
        img_msg.header.stamp = self.get_clock().now().to_msg()

        start = time.time()
        while time.time() - start < timeout_sec:
            # Publish image periodically
            self.pub_cam.publish(img_msg)
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.received_detection is not None and self.received_annotated_img is not None:
                return True
        return False


def main():
    rclpy.init()
    test_img = "dataset/combined_dataset.v1i.yolov8/test/images/000035_jpg.rf.5486bab07af640a8c9b063cbbc91607c.jpg"
    tester = PerceptionTester(test_img)

    print(f"Feeding image {test_img} into perception pipeline...")
    success = tester.run_test(timeout_sec=4.0)

    if tester.received_detection:
        print("\n=== SUCCESS: RECEIVED LIVE 2D DETECTION MESSAGE ===")
        for d in tester.received_detection.detections:
            for r in d.results:
                print(f"Class: {r.hypothesis.class_id}")
                print(f"Confidence: {r.hypothesis.score:.3f}")
            print(f"BBox center: ({d.bbox.center.position.x:.1f}, {d.bbox.center.position.y:.1f})")
            print(f"BBox size: {d.bbox.size_x:.1f} x {d.bbox.size_y:.1f}")

    if tester.received_annotated_img is not None:
        out_path = Path(".sage/render/test_detection_output.jpg")
        cv2.imwrite(str(out_path), tester.received_annotated_img)
        print(f"\nSaved annotated detection image to: {out_path}")

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
