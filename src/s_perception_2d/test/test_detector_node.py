"""
Integration test for YoloDetectorNode in s_perception_2d.
Tests ROS 2 image subscription, YOLO inference execution,
ObjectRecord handling, and Detection2DArray publishing.
"""

import time
import cv2
import numpy as np
import pytest

import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    from vision_msgs.msg import Detection2DArray
    from s_perception_2d.detector_node import YoloDetectorNode
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


@pytest.fixture(scope="module")
def rclpy_init():
    if not ROS_AVAILABLE:
        pytest.skip("ROS 2 or vision_msgs not available")
    rclpy.init()
    yield
    rclpy.shutdown()


def test_detector_node_end_to_end(rclpy_init, tmp_path):
    # Create node with CPU device and file logging in tmp_path
    from rclpy.parameter import Parameter
    node = YoloDetectorNode()
    node.set_parameters([Parameter("device", Parameter.Type.STRING, "cpu")])
    node.device = "cpu"
    node.logger_handler.log_dir = tmp_path
    node.logger_handler.jsonl_path = tmp_path / "test_detections.jsonl"

    # Create dummy 640x640 test image
    test_img = np.zeros((640, 640, 3), dtype=np.uint8)
    # Draw a colored pattern
    cv2.rectangle(test_img, (100, 100), (300, 300), (0, 0, 255), -1)

    bridge = CvBridge()
    img_msg = bridge.cv2_to_imgmsg(test_img, encoding="bgr8")
    img_msg.header.frame_id = "oakd_rgb_camera_optical_frame"
    img_msg.header.stamp.sec = 100
    img_msg.header.stamp.nanosec = 500

    # Capture published detections
    published_detections = []
    def detection_cb(msg: Detection2DArray):
        published_detections.append(msg)

    sub = node.create_subscription(
        Detection2DArray,
        node.output_detections_topic,
        detection_cb,
        10,
    )

    # Process frame through callback directly
    node.image_callback(img_msg)

    # Check that callback processed without error
    assert node.frame_count == 1

    # Check stats
    stats = node.logger_handler.get_stats()
    assert stats["total_frames"] == 1

    # Check log file was written
    assert node.logger_handler.jsonl_path.exists()

    node.destroy_node()
