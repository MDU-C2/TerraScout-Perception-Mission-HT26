#!/usr/bin/env python3
"""
Test utility node to stream sample dataset images to a ROS 2 camera topic.
Allows testing detection and Foxglove visualization without needing 3D models in Gazebo.
"""

import os
import glob
import time
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageStreamerNode(Node):
    def _declare_param(self, name, default):
        return getattr(self, "declare_parameter")(name, default)

    def __init__(self):
        super().__init__('image_streamer_node')

        # Search upwards for dataset directory
        cur = Path(__file__).resolve().parent
        ws_root = cur
        for p in [cur] + list(cur.parents):
            if (p / "dataset").is_dir():
                ws_root = p
                break

        default_dir = ws_root / "dataset" / "Car-Silhouettes-Forest.v1i.yolov8" / "test" / "images"
        if not default_dir.exists():
            default_dir = ws_root / "dataset" / "test" / "images"

        self._declare_param('topic', '/terrascout1/oakd/rgb/image_raw')
        self._declare_param('rate_hz', 2.0)
        self._declare_param('image_dir', str(default_dir))

        self.topic_name = self.get_parameter('topic').get_parameter_value().string_value
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.image_dir = self.get_parameter('image_dir').get_parameter_value().string_value

        self.publisher = self.create_publisher(Image, self.topic_name, 10)
        self.bridge = CvBridge()

        # Find test images (supports directory or single image file)
        if os.path.isfile(self.image_dir):
            self.image_files = [self.image_dir]
        elif os.path.isdir(self.image_dir):
            self.image_files = sorted(glob.glob(os.path.join(self.image_dir, '*.jpg'))) + \
                               sorted(glob.glob(os.path.join(self.image_dir, '*.png')))
        else:
            self.image_files = []

        if not self.image_files:
            self.get_logger().error(f"No images found in {self.image_dir}")
            return

        self.get_logger().info(
            f"Found {len(self.image_files)} images. Streaming to '{self.topic_name}' at {self.rate_hz} Hz."
        )

        self.current_idx = 0
        timer_period = 1.0 / max(0.1, self.rate_hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if not self.image_files:
            return

        img_path = self.image_files[self.current_idx]
        cv_img = cv2.imread(img_path)

        if cv_img is not None:
            # Resize standard resolution if huge
            msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'oakd_rgb_camera_optical_frame'
            self.publisher.publish(msg)
            self.get_logger().info(f"[{self.current_idx + 1}/{len(self.image_files)}] Published: {os.path.basename(img_path)}")

        self.current_idx = (self.current_idx + 1) % len(self.image_files)


def main(args=None):
    rclpy.init(args=args)
    node = ImageStreamerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
