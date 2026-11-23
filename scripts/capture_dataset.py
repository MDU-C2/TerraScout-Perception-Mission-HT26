#!/usr/bin/env python3
"""
Dataset Image Collector from ROS 2 Camera Topic
"""

import argparse
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False


class ImageCollector(Node):
    def __init__(self, topic: str, output_dir: str, interval: float, prefix: str):
        super().__init__('dataset_image_collector')
        self.topic = topic
        self.output_dir = Path(output_dir).resolve()
        self.interval = interval
        self.prefix = prefix
        self.bridge = CvBridge() if HAS_CV_BRIDGE else None

        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Find existing highest index so we don't overwrite existing captures
        existing = list(self.output_dir.glob(f"{self.prefix}_*.jpg"))
        self.counter = len(existing) + 1
        self.last_saved_time = 0.0

        self.subscription = self.create_subscription(
            Image,
            self.topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f"Subscribed to topic: {self.topic}")
        self.get_logger().info(f"Saving images to:   {self.output_dir}")
        self.get_logger().info(f"Capture interval:  {self.interval}s")
        self.get_logger().info("Drive the robot around! Press Ctrl+C to stop.\n")

    def image_callback(self, msg: Image):
        now = time.time()
        if now - self.last_saved_time < self.interval:
            return

        self.last_saved_time = now

        try:
            if self.bridge is not None:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                # Direct numpy fallback
                if msg.encoding in ('bgr8', 'rgb8'):
                    cv_img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                    if msg.encoding == 'rgb8':
                        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
                else:
                    self.get_logger().warn(f"Unsupported encoding: {msg.encoding}")
                    return

            filename = self.output_dir / f"{self.prefix}_{self.counter:04d}.jpg"
            cv2.imwrite(str(filename), cv_img)

            h, w = cv_img.shape[:2]
            print(f"[{self.counter:04d}] Saved: {filename.name} ({w}x{h})")
            self.counter += 1

        except Exception as e:
            self.get_logger().error(f"Failed to convert/save image: {e}")


def main():
    parser = argparse.ArgumentParser(description="Capture raw images from ROS 2 camera for YOLO dataset")
    parser.add_argument(
        "--topic",
        default="/terrascout1/oakd/rgb/image_raw",
        help="ROS 2 camera topic (default: /terrascout1/oakd/rgb/image_raw)"
    )
    parser.add_argument(
        "--output-dir",
        default="/workspace/src/dataset_sim",
        help="Directory to save captured images (default: /workspace/src/dataset_sim)"
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Interval in seconds between captured frames (default: 1.0s)"
    )
    parser.add_argument(
        "--prefix",
        default="sim",
        help="Image file prefix (default: 'sim' -> sim_0001.jpg)"
    )
    args = parser.parse_args()

    rclpy.init()
    collector = ImageCollector(
        topic=args.topic,
        output_dir=args.output_dir,
        interval=args.interval,
        prefix=args.prefix
    )

    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        print(f"\nStopped capture. Total images in folder: {collector.counter - 1}")
    finally:
        collector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

