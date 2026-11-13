#!/usr/bin/env python3
"""
Test runner for Workflow B: Offline Image Streamer + YOLO Detector.
Streams images from test dataset, verifies detections, and saves an annotated frame.
"""

import os
import sys
import time
import subprocess
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge


class WorkflowBVerifier(Node):
    def __init__(self, output_save_path):
        super().__init__('workflow_b_verifier')
        self.output_save_path = output_save_path
        self.bridge = CvBridge()
        self.received_detections = []
        self.saved_image = False

        self.sub_detections = self.create_subscription(
            Detection2DArray,
            '/terrascout1/oakd/detections_2d',
            self.detections_callback,
            10,
        )

        self.sub_annotated = self.create_subscription(
            Image,
            '/terrascout1/oakd/detections_image',
            self.annotated_callback,
            10,
        )

    def detections_callback(self, msg: Detection2DArray):
        for det in msg.detections:
            for hyp in det.results:
                rec_info = {
                    'class_id': hyp.hypothesis.class_id,
                    'score': hyp.hypothesis.score,
                    'center_u': det.bbox.center.position.x,
                    'center_v': det.bbox.center.position.y,
                    'width': det.bbox.size_x,
                    'height': det.bbox.size_y,
                }
                self.received_detections.append(rec_info)
                print(f"[DETECTION RECEIVED] Class: '{rec_info['class_id']}' | Conf: {rec_info['score']:.3f} | Center: ({rec_info['center_u']:.1f}, {rec_info['center_v']:.1f}) | Size: {rec_info['width']:.1f}x{rec_info['height']:.1f}")

    def annotated_callback(self, msg: Image):
        if not self.saved_image and len(self.received_detections) > 0:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(self.output_save_path, cv_img)
            self.saved_image = True
            print(f"[IMAGE SAVED] Annotated image saved to: {self.output_save_path}")


def main():
    save_path = "/home/sadra/.gemini/antigravity-ide/brain/647d52bb-4f85-4323-83a8-72a97c940c22/workflow_b_detection_sample.jpg"
    rclpy.init()
    node = WorkflowBVerifier(save_path)

    print("Listening for detections and annotated frames from Workflow B...")
    start_time = time.time()
    while rclpy.ok() and (time.time() - start_time < 12.0):
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.saved_image and len(node.received_detections) >= 3:
            print("Successfully verified multiple detections and captured annotated image!")
            break

    print(f"\n--- Verification Summary ---")
    print(f"Total detections collected: {len(node.received_detections)}")
    for i, d in enumerate(node.received_detections[:5]):
        print(f"  {i+1}. Class: {d['class_id']} (conf={d['score']:.2f}), Center: ({d['center_u']:.1f}, {d['center_v']:.1f})")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if len(node.received_detections) > 0 else 1


if __name__ == '__main__':
    sys.exit(main())
